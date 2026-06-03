#pragma once
#include "provider.h"

namespace notify {

// Telegram provider — sends via POST to api.telegram.org/bot<token>/sendMessage.
// Config fields: notify_telegram_enabled, notify_telegram_token, notify_telegram_chat_id.
class TelegramProvider : public INotifyProvider {
public:
  const char* id()                               const override { return "telegram"; }
  bool        is_enabled(const Config& cfg)       const override;
  bool        send(const NotifyMessage& msg,
                   const Config&        cfg,
                   char*                err_out,
                   size_t               err_out_size) const override;
};

}  // namespace notify
