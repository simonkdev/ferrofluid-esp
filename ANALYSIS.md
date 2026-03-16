# ESP32-CAM_MJPEG2SD Codebase Findings (Current Config)

## High-level architecture
- **Startup flow** (`ESP32-CAM_MJPEG2SD.ino`): initializes logging, storage, configuration, camera, network, web server, and then starts sustain/recording tasks. Setup ends by spawning long‑running tasks and deleting the main loop task.
- **Core capture/recording pipeline** (`mjpeg2sd.cpp`, `avi.cpp`):
  - Captures JPEG frames from the camera and writes them into AVI containers aligned to SD sector boundaries.
  - Supports motion‑triggered recording, manual recording, time‑lapse, and continuous (dashcam) capture.
  - Playback reads AVI frames and streams them back to the browser at recorded FPS.
- **Motion detection** (`motionDetect.cpp`): performs frame differencing and banded thresholding to trigger motion events; controls recording triggers and motion‑based night mode switching.
- **Web server + UI** (`webServer.cpp`, `data/MJPEG2SD.htm`, `data/common.js`):
  - HTTP endpoints for status, control, file management, OTA, log access, and streaming.
  - Browser UI builds configuration panels dynamically from a config table and live status JSON.
- **Configuration + persistence** (`prefs.cpp`):
  - Loads/saves a config vector from SD/LittleFS.
  - Stores passwords in NVS and merges them into runtime config.
  - Provides status/config JSON to the UI and processes updates from the browser.
- **Shared utilities** (`utils.cpp`, `utilsFS.cpp`, `setupAssist.cpp`):
  - Time/NTP, storage management, OTA support, logging, network helpers, and data file bootstrap.

## Features currently disabled in `appGlobals.h`
These compile‑time feature switches are all set to `false`, so their implementations are unused in the current build:
- FTP/HTTPS upload, Telegram, Audio, Peripherals, SMTP, MQTT/Home Assistant
- HTTPS cert validation, UART auxiliary, Telemetry, WebDAV
- External heartbeat, Photogrammetry, MCPWM motor control
- RTSP streaming, DS18B20 sensor, OV5640 auto‑focus
- esp_new_jpg, I2C peripheral support, TinyML

## Implication for cleanup
Because the optional features above are disabled, the currently active build only uses:
- Core camera capture/AVI recording + playback
- Motion detection (camera‑based)
- HTTP streaming + web UI
- Config storage + OTA + logging
Removing unused feature code paths should simplify the codebase and reduce UI/config surface area without changing core behavior.
