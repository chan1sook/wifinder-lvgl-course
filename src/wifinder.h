#ifndef _WIFINDER_LVGL_H
#define _WIFINDER_LVGL_H

#include <Arduino.h>
#include <lvgl.h>

#define BUZZER_GPIO (33)
#define BUZZER_ENABLED 1
#define MIN_RSSI_VALUE (-120)

const char *WIFINDER_LOGGING PROGMEM = "WIFINDER";

const char *START_WIFI_SCAN_TEXT PROGMEM = "Start Scan";
const char *STOP_WIFI_SCAN_TEXT PROGMEM = "Stop Scan";

const char *START_RSSI_SCAN_TEXT PROGMEM = "Select WiFi";
const char *STOP_RSSI_SCAN_TEXT PROGMEM = "Deselect WiFi";

const char *RSSI_EMPTY_VALUE PROGMEM = "-";

enum wifinder_scan_state
{
  WIFINDER_IDLE,
  WIFINDER_SCAN_WIFI,
  WIFINDER_SCAN_RSSI,
};

struct wifiner_scan_flag
{
  bool scanWifi : 1;
  bool scanRssi : 1;
};

#endif