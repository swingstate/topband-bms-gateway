# Third-Party Notices

This file lists the third-party open-source components used in the TopBand BMS
Gateway firmware, together with their licenses and attribution requirements.

---

## TopBand BMS protocol byte layout

Reverse-engineering research by **linedot**:
<https://github.com/linedot/topbands-bms>

The implementation in `src/bms/protocol.cpp` is an independent clean rewrite
for ESP-IDF. The byte-layout documentation and enum naming from linedot's work
were used as reference. License: MIT (Copyright (c) 2026 linedot).
Attribution included in `src/bms/protocol.h` file header.

---

## microtar

MIT License  
Copyright (c) 2017 rxi  
<https://github.com/rxi/microtar>

Used for extracting the embedded LittleFS UI bundle (`littlefs_ui.tar.gz`) on
first boot. Vendored in `src/third_party/microtar/`.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions: The above copyright notice and this
permission notice shall be included in all copies or substantial portions of
the Software.

---

## tinfl (IDF embedded gzip/deflate decompressor)

Public domain (Miniz, by Rich Geldreich)  
Included as part of ESP-IDF's `esp_rom` component.

Used alongside microtar for decompressing `littlefs_ui.tar.gz`.

---

## ESP-IDF

Apache License 2.0  
Copyright 2015-2024 Espressif Systems  
<https://github.com/espressif/esp-idf>

The firmware is built on and statically linked against ESP-IDF. Full license
text at the ESP-IDF repository.

---

## ArduinoJson v7

MIT License  
Copyright 2014-2024 Benoit Blanchon  
<https://arduinojson.org>

Used for JSON serialization in HTTP handlers and MQTT payloads (Phases F/H).
Included via PlatformIO managed library.

---

## Catch2 v3

BSL-1.0 License  
Copyright 2021 Two Blue Cubes Ltd.  
<https://github.com/catchorg/Catch2>

Used exclusively in host-side unit tests (`test/host/`). Not linked into the
firmware binary.
