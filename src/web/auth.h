#pragma once
#include "esp_http_server.h"
#include <string>

// ── Auth module ───────────────────────────────────────────────────────────────
// Double-submit cookie + CSRF protection.  Architecture §6.2, D2.3, D3.3.
//
// Session token: 32-hex-char random string, generated fresh on every boot
// (RAM-only per D2.3 — server reboot invalidates all sessions by design).
//
// Cookie:  tbsid=<token>; HttpOnly; SameSite=Lax; Path=/; Max-Age=2592000 (30 days)
// CSRF:    Same token injected as <meta name="csrf-token"> in index.html.
//          Mutating requests (POST/PUT/DELETE/PATCH) must include it in the
//          X-CSRF-Token header (double-submit pattern).
//
// Auth bypass: when cfg.auth_enabled == false (after 5x reset or before first
// password is set), check() returns true unconditionally so the UI is
// accessible without credentials.

namespace web::auth {

// Generate fresh session token from hardware RNG. Call once per boot.
void init();

// Returns true if the request passes auth checks.
// GET requests:  cookie tbsid == session token.
// POST/…:        cookie match AND X-CSRF-Token header match.
// auth_enabled == false: always true.
bool check(httpd_req_t* req);

// POST /api/auth/login — body: {"password":"..."}
// On success: Set-Cookie, return {"ok":true,"csrf":"<token>"}.
// On wrong password: 401 + 50 ms delay.
esp_err_t handler_login(httpd_req_t* req);

// POST /api/auth/logout — clears cookie via Max-Age=0, returns {"ok":true}.
esp_err_t handler_logout(httpd_req_t* req);

// POST /api/auth/set_password — body: {"current":"...","new":"..."}
// When auth_enabled == false: "current" is ignored (initial setup).
// On success: persists hashed password, sets auth_enabled=true.
esp_err_t handler_set_password(httpd_req_t* req);

// Return the current CSRF token (points to an internal buffer; valid until
// next init() call). Used by handlers_static to inject into index.html.
const char* get_csrf_token();

// SHA-256 of plain, returned as 64 lowercase hex chars (+ '\0').
std::string hash_password(const std::string& plain);

}  // namespace web::auth
