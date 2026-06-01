#pragma once
#include "esp_http_server.h"

namespace web {

// POST /api/notify/telegram/test
// Body: {notify_telegram_enabled, notify_telegram_token, notify_telegram_chat_id}
// Kicks off an off-thread test send using the (possibly unsaved) form values.
// Returns 200 immediately; poll GET for the result.
esp_err_t handle_notify_telegram_test_post(httpd_req_t* req);

// GET /api/notify/telegram/test
// Returns {status:"idle"|"running"|"ok"|"failed", message:"..."}
esp_err_t handle_notify_telegram_test_get(httpd_req_t* req);

}  // namespace web
