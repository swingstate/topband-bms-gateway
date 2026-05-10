#include "captdns.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdint>

static const char* TAG = "captdns";

static volatile bool g_running = false;
static int          g_sock     = -1;
static TaskHandle_t g_task     = nullptr;

// ── Minimal RFC 1035 DNS response builder ─────────────────────────────────────
//
// Wire format for a DNS A-record response:
//   Header  (12 bytes): ID, Flags, QDCount=1, ANCount=1, NSCount=0, ARCount=0
//   Question section: copied verbatim from the query (QNAME + QTYPE + QCLASS)
//   Answer RR: NAME ptr(C00C) | TYPE=A | CLASS=IN | TTL=60 | RDLEN=4 | IPv4
//
// We copy the entire incoming query packet, overwrite the header fields, then
// append the answer record at the end. The name pointer 0xC00C (bits 11=1 +
// offset 12) references the QNAME in the question section at byte 12.

static void dns_task(void* arg) {
  uint32_t reply_ip = (uint32_t)(uintptr_t)arg;

  struct sockaddr_in addr = {};
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(53);
  addr.sin_addr.s_addr = INADDR_ANY;

  g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (g_sock < 0) {
    ESP_LOGE(TAG, "socket() failed: errno %d", errno);
    g_running = false;
    vTaskDelete(nullptr);
    return;
  }

  // 1-second receive timeout so we can check g_running periodically.
  struct timeval tv = {};
  tv.tv_sec = 1;
  setsockopt(g_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (bind(g_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "bind(53) failed: errno %d", errno);
    closesocket(g_sock);
    g_sock    = -1;
    g_running = false;
    vTaskDelete(nullptr);
    return;
  }

  uint8_t in_ip[4];
  memcpy(in_ip, &reply_ip, 4);
  ESP_LOGI(TAG, "DNS server listening on :53 → %u.%u.%u.%u",
           in_ip[0], in_ip[1], in_ip[2], in_ip[3]);

  uint8_t buf[512];
  uint8_t resp[512];

  while (g_running) {
    struct sockaddr_in client = {};
    socklen_t client_len = sizeof(client);
    int n = recvfrom(g_sock, buf, sizeof(buf), 0,
                     (struct sockaddr*)&client, &client_len);
    if (n < 0) continue;  // EAGAIN (timeout) or error — loop
    if (n < 12) continue; // too short for a DNS header

    // ── Inspect query header ──────────────────────────────────────────────
    uint16_t flags = (uint16_t)((buf[2] << 8) | buf[3]);
    // QR=0 means query, OPCODE must be 0 (standard query).
    if ((flags & 0x8000u) || ((flags >> 11) & 0xFu) != 0) continue;

    // ── Build response ────────────────────────────────────────────────────
    // Copy the entire incoming packet (header + question section) first.
    if (n > (int)sizeof(resp) - 16) continue;  // no room for answer — skip
    memcpy(resp, buf, (size_t)n);

    // Overwrite header flags: QR=1 AA=1 RA=1 RCODE=0
    resp[2] = 0x81;
    resp[3] = 0x80;
    // QDCount stays 1 (already copied from query).
    // ANCount = 1
    resp[6] = 0x00;
    resp[7] = 0x01;
    // NSCount = 0, ARCount = 0
    resp[8] = resp[9] = resp[10] = resp[11] = 0x00;

    // Append answer RR after the question section (which ends at byte n).
    int off = n;
    // NAME: pointer to QNAME at offset 12 (0xC00C in big-endian)
    resp[off++] = 0xC0;
    resp[off++] = 0x0C;
    // TYPE A = 1
    resp[off++] = 0x00;
    resp[off++] = 0x01;
    // CLASS IN = 1
    resp[off++] = 0x00;
    resp[off++] = 0x01;
    // TTL = 60 s
    resp[off++] = 0x00;
    resp[off++] = 0x00;
    resp[off++] = 0x00;
    resp[off++] = 0x3C;
    // RDLENGTH = 4
    resp[off++] = 0x00;
    resp[off++] = 0x04;
    // RDATA = IP address (network byte order)
    memcpy(resp + off, &reply_ip, 4);
    off += 4;

    sendto(g_sock, resp, (size_t)off, 0,
           (struct sockaddr*)&client, client_len);
  }

  closesocket(g_sock);
  g_sock = -1;
  ESP_LOGI(TAG, "DNS server stopped");
  vTaskDelete(nullptr);
}

namespace net::captdns {

bool start(uint32_t ip_to_return) {
  g_running = true;
  BaseType_t ret = xTaskCreate(dns_task, "captdns", 4096,
                               (void*)(uintptr_t)ip_to_return, 5, &g_task);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "xTaskCreate failed");
    g_running = false;
    return false;
  }
  return true;
}

void stop() {
  g_running = false;
  // Give the task up to 1.5 s to notice g_running=false and exit.
  vTaskDelay(pdMS_TO_TICKS(1500));
  g_task = nullptr;
}

}  // namespace net::captdns
