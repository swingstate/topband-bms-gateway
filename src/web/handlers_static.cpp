#include "handlers_static.h"
#include "net/wifi.h"
#include "storage/lfs_store.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "web_static";

// Extension → MIME-type lookup.
static const char* mime_for(const char* path) {
  const char* ext = strrchr(path, '.');
  if (!ext) return "application/octet-stream";
  if (strcmp(ext, ".html") == 0) return "text/html; charset=utf-8";
  if (strcmp(ext, ".css")  == 0) return "text/css";
  if (strcmp(ext, ".js")   == 0) return "application/javascript";
  if (strcmp(ext, ".json") == 0) return "application/json";
  if (strcmp(ext, ".svg")  == 0) return "image/svg+xml";
  if (strcmp(ext, ".ico")  == 0) return "image/x-icon";
  if (strcmp(ext, ".png")  == 0) return "image/png";
  if (strcmp(ext, ".txt")  == 0) return "text/plain";
  return "application/octet-stream";
}

static const char WIFI_SETUP_HTML[] =
  "<!DOCTYPE html><html><head>"
  "<meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>TopBand BMS Gateway — WiFi Setup</title>"
  "<style>"
  "body{font-family:system-ui,sans-serif;background:#0f0f0f;color:#f5f5f5;"
  "display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}"
  ".card{background:#1a1a1a;border:1px solid #333;border-radius:12px;padding:2rem;"
  "width:340px;box-shadow:0 4px 24px #0008}"
  "h1{font-size:1.2rem;margin:0 0 1.5rem;color:#f5f5f5}"
  "label{display:block;font-size:.85rem;color:#aaa;margin-bottom:.3rem}"
  "input{width:100%;box-sizing:border-box;background:#111;border:1px solid #444;"
  "border-radius:6px;color:#f5f5f5;padding:.5rem .75rem;font-size:1rem;margin-bottom:1rem}"
  "button{width:100%;background:#22c55e;border:none;border-radius:6px;color:#fff;"
  "font-size:1rem;font-weight:600;padding:.65rem;cursor:pointer}"
  "button:hover{background:#16a34a}"
  ".info{font-size:.8rem;color:#666;margin-top:1rem;text-align:center}"
  "</style></head><body>"
  "<div class=card>"
  "<h1>TopBand BMS Gateway<br><span style='font-weight:400;color:#aaa'>WiFi Setup</span></h1>"
  "<label>WiFi Network (SSID)</label>"
  "<input id=ssid type=text autocomplete=off placeholder='Enter SSID'>"
  "<label>Password</label>"
  "<input id=pass type=password autocomplete=off placeholder='Enter password'>"
  "<button onclick=save()>Connect &amp; Restart</button>"
  "<p class=info>After saving the gateway reboots and joins your network.</p>"
  "</div>"
  "<script>"
  "function save(){"
  "var s=document.getElementById('ssid').value;"
  "var p=document.getElementById('pass').value;"
  "if(!s){alert('SSID required');return;}"
  "fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},"
  "body:JSON.stringify({ssid:s,pass:p})})"
  ".then(r=>r.json()).then(d=>{"
  "document.body.innerHTML='<div style=\"color:#22c55e;font-family:system-ui;text-align:center;"
  "margin-top:40vh\">Connecting to '+s+'…<br>Gateway is rebooting.</div>';"
  "}).catch(e=>alert('Error: '+e));}"
  "</script></body></html>";

namespace web {

esp_err_t handle_wifi_setup_page(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_sendstr(req, WIFI_SETUP_HTML);
}

esp_err_t handle_static(httpd_req_t* req) {
  const char* uri = req->uri;

  // AP mode: serve the WiFi setup page for all paths.
  if (net::wifi::is_ap_mode()) {
    return handle_wifi_setup_page(req);
  }

  // Map URI to LittleFS path. Buffer is 600 B: URI max 512 + "/lfs/ui" prefix
  // 7 + null = 520 max output. 600 gives GCC enough headroom to prove no
  // truncation without a runtime length check.
  char lfs_path[600];
  if (strcmp(uri, "/") == 0 || strcmp(uri, "/index.html") == 0) {
    snprintf(lfs_path, sizeof(lfs_path), "/lfs/ui/index.html");
  } else {
    snprintf(lfs_path, sizeof(lfs_path), "/lfs/ui%s", uri);
  }

  // Security: reject paths with ".." traversal.
  if (strstr(lfs_path, "..")) {
    httpd_resp_set_status(req, "403 Forbidden");
    return httpd_resp_sendstr(req, "Forbidden");
  }

  if (!storage::lfs::exists(lfs_path)) {
    ESP_LOGD(TAG, "static: not found: %s", lfs_path);
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"Not found\"}");
  }

  // Read file and stream to client.
  // Files > 2 KB use chunked transfer. The chunk size limits peak RAM.
  FILE* f = fopen(lfs_path, "rb");
  if (!f) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"File open failed\"}");
  }

  const char* mime = mime_for(lfs_path);
  httpd_resp_set_type(req, mime);

  // Cache-Control: no-cache for index.html; 1 hour for everything else.
  bool is_index = (strcmp(lfs_path, "/lfs/ui/index.html") == 0);
  httpd_resp_set_hdr(req, "Cache-Control",
                     is_index ? "no-cache, must-revalidate" : "max-age=3600");

  static constexpr size_t CHUNK = 2048;
  char* buf = (char*)malloc(CHUNK);
  if (!buf) {
    fclose(f);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }

  size_t n;
  esp_err_t ret = ESP_OK;
  while ((n = fread(buf, 1, CHUNK, f)) > 0) {
    ret = httpd_resp_send_chunk(req, buf, (ssize_t)n);
    if (ret != ESP_OK) {
      ESP_LOGD(TAG, "client disconnected during static send: %s", lfs_path);
      break;
    }
  }
  fclose(f);
  free(buf);

  if (ret == ESP_OK) {
    httpd_resp_send_chunk(req, NULL, 0);  // Terminate chunked response.
  }
  return ret;
}

}  // namespace web
