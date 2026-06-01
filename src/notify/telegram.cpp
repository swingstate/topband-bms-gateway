#include "telegram.h"
#include "net/http_post.h"
#include "esp_log.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "notify_tg";

// Maximum URL: https://api.telegram.org/bot<token>/sendMessage
// Telegram bot tokens are ~50 chars; allow up to 120 for the token part.
static constexpr int TG_TIMEOUT_MS = 10000;

namespace {

// Severity label for the Telegram message prefix.
static const char* severity_label(notify::Severity s) {
  switch (s) {
    case notify::Severity::Warning:  return "[WARNING] ";
    case notify::Severity::Critical: return "[CRITICAL] ";
    default:                         return "[INFO] ";
  }
}

// Escape characters that would break a JSON string value.
// Writes into out[max], returns chars written (excl. null).
static size_t json_escape(const char* src, char* out, size_t max) {
  size_t n = 0;
  for (; *src && n < max - 2; ++src) {
    char c = *src;
    if (c == '"' || c == '\\') {
      if (n + 2 >= max) break;
      out[n++] = '\\';
    }
    out[n++] = c;
  }
  out[n] = '\0';
  return n;
}

}  // namespace

namespace notify {

bool TelegramProvider::is_enabled(const Config& cfg) const {
  return cfg.notify_telegram_enabled
      && cfg.notify_telegram_token[0]  != '\0'
      && cfg.notify_telegram_chat_id[0] != '\0';
}

bool TelegramProvider::send(const NotifyMessage& msg,
                             const Config&        cfg,
                             char*                err_out,
                             size_t               err_out_size) const {
  if (cfg.notify_telegram_token[0] == '\0') {
    snprintf(err_out, err_out_size, "Bot token is empty — enter a token first");
    return false;
  }
  if (cfg.notify_telegram_chat_id[0] == '\0') {
    snprintf(err_out, err_out_size, "Chat ID is empty — enter a chat ID first");
    return false;
  }

  // Build the API URL.
  char url[192];
  snprintf(url, sizeof(url),
           "https://api.telegram.org/bot%s/sendMessage",
           cfg.notify_telegram_token);

  // Build the message text: severity prefix + title + newline + body.
  char text_raw[320];
  snprintf(text_raw, sizeof(text_raw), "%s%s\n%s",
           severity_label(msg.severity),
           msg.title ? msg.title : "",
           msg.body  ? msg.body  : "");

  char text_esc[400];
  json_escape(text_raw, text_esc, sizeof(text_esc));

  char chat_esc[32];
  json_escape(cfg.notify_telegram_chat_id, chat_esc, sizeof(chat_esc));

  // JSON body for sendMessage.
  char body[512];
  snprintf(body, sizeof(body),
           "{\"chat_id\":\"%s\",\"text\":\"%s\"}",
           chat_esc, text_esc);

  net::HttpPostResult r = net::https_post(url, body, nullptr, TG_TIMEOUT_MS);

  if (r.success) {
    ESP_LOGI(TAG, "Telegram send OK (HTTP %d)", r.http_status);
    return true;
  }

  // Map known HTTP error codes to clear messages.
  if (r.http_status == 401) {
    snprintf(err_out, err_out_size,
             "Invalid bot token — check the token from BotFather (HTTP 401)");
  } else if (r.http_status == 400) {
    snprintf(err_out, err_out_size,
             "Chat not found — check the Chat ID (HTTP 400). "
             "Use @userinfobot or send a message to your bot first.");
  } else if (r.http_status == 403) {
    snprintf(err_out, err_out_size,
             "Bot was blocked or chat doesn't exist (HTTP 403)");
  } else if (r.http_status > 0) {
    snprintf(err_out, err_out_size,
             "Telegram API error (HTTP %d)", r.http_status);
  } else {
    // No HTTP response — network/TLS failure.
    snprintf(err_out, err_out_size, "Network/TLS error: %s", r.error);
  }

  ESP_LOGW(TAG, "Telegram send failed: %s", err_out);
  return false;
}

}  // namespace notify
