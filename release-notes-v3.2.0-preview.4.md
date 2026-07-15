# v3.2.0-preview.4

Bugfix release for Pylontech/Deye CAN users and MQTT/Home Assistant users.

- Fixed a false "undertemperature" alarm over Pylontech CAN that could block
  charging on a healthy, room-temperature battery.
- Per-pack Current, SOC, and Power now publish correctly over MQTT (previously
  showed as "Unknown" in Home Assistant for some setups).
- Fixed an inconsistency where the aggregate SOC over MQTT could briefly
  disagree with the dashboard and individual pack values.
- Fixed repeated Home Assistant log warnings for solar/MPPT sensors when data
  is temporarily unavailable.

No configuration changes needed. Safe to update from v3.2.0-preview.3.
