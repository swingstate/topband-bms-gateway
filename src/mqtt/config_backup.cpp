#include "mqtt/config_backup.h"
#include "web/config_json.h"

namespace mqtt::config_backup {

void build_json(const Config& cfg, JsonDocument& doc) {
  web::config_to_json(cfg, doc);
  doc.remove("wifi_ssid");
  doc.remove("mqtt_host");
  doc.remove("mqtt_port");
  doc.remove("mqtt_user");
  doc.remove("mqtt_pass_obf");
  doc.remove("auth_user");
}

}  // namespace mqtt::config_backup
