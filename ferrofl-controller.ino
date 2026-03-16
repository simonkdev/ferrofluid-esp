#include "appGlobals.h"

void setup() {
  logSetup();
  LOG_INF("Selected board %s", CAM_BOARD);
  // prep storage
  if (startStorage()) {
    // Load saved user configuration
    if (loadConfig()) {
#ifndef AUXILIARY
      // initialise camera
      if (psramFound()) {
        if (ESP.getPsramSize() > 1 * ONEMEG) {
          prepCam();
        } else {
          snprintf(startupFailure, SF_LEN, STARTUP_FAIL "Insufficient PSRAM for app: %s", fmtSize(ESP.getPsramSize()));
        }
      } else {
        snprintf(startupFailure, SF_LEN, STARTUP_FAIL "Need PSRAM to be enabled");
      }
#else
      LOG_INF("AUXILIARY mode without camera");
#endif
    }
  }

#ifdef DEV_ONLY
  devSetup();
#endif

  startNetwork();
  if (startWebServer()) {
    // start rest of services

#ifndef AUXILIARY
    startSustainTasks();
#endif

#ifndef AUXILIARY
    if (!prepRecording()) {
      snprintf(startupFailure, SF_LEN, STARTUP_FAIL "Insufficient memory, remove optional features");
      LOG_WRN("%s", startupFailure);
    }
#endif
    checkMemory();
  }
}

void loop() {
  // confirm not blocked in setup
  LOG_INF("=============== Total tasks: %u ===============\n", uxTaskGetNumberOfTasks() - 1);
  delay(1000);
  vTaskDelete(NULL);  // free 8k ram
}
