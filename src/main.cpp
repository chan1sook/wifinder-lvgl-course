#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#include <SPI.h>
#include <TFT_eSPI.h>
#include "FT62XXTouchScreen.h"

#include "wifinder.h"

#include "esp_freertos_hooks.h"
#include "ui/ui.h"

static const uint16_t screenWidth = TFT_WIDTH;
static const uint16_t screenHeight = TFT_HEIGHT;

TFT_eSPI tft = TFT_eSPI();
FT62XXTouchScreen touchScreen = FT62XXTouchScreen(screenHeight, PIN_SDA, PIN_SCL);

#define BUFFER_SIZE (screenWidth * screenHeight / 10)

static lv_disp_draw_buf_t disp_buf;
static lv_color_t *screenBuffer1;

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

wifiner_scan_flag wifinderFlags = {false, false};
#define SSID_MAX_LENGTH (33)
char targetWifiSSID[SSID_MAX_LENGTH];

uint32_t lastestRSSI;
uint32_t lastestAlarmTs;

static void fullUpdateScreen();

void toggleWifiScan();
static void beginWifiScan();
static void stopWifiScan();

void toggleSelectWifi();
static void beginRssiScan();
static void stopRssiScan();

static uint16_t getWifiOptionsLength();
static lv_color_t getRssiColor(int32_t rssi);

static void updateRSSIArc(int32_t rssi);
static void emitAlarm(int32_t rssi);

void setup()
{
  pinMode(BUZZER_GPIO, OUTPUT);

  Serial.begin(115200);

  // Init LVGL
  lv_init();

  // Enable TFT
  tft.begin();
  tft.setRotation(1);

  // Enable Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, 1);

  // Start TouchScreen
  touchScreen.begin();

  // Display Buffer
  screenBuffer1 = (lv_color_t *)ps_malloc(BUFFER_SIZE * sizeof(lv_color_t));
  lv_disp_draw_buf_init(&disp_buf, screenBuffer1, NULL, BUFFER_SIZE);

  // Initialize the display
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  // Init Touchscreen
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);

  ui_init();

  lv_roller_set_options(ui_WifiSSIDListRoller, "", LV_ROLLER_MODE_NORMAL);
  updateRSSIArc(MIN_RSSI_VALUE);
  lv_label_set_text(ui_RSSILabel, RSSI_EMPTY_VALUE);
  fullUpdateScreen();
}

void loop()
{
  int16_t wifiCounts = WiFi.scanComplete();
  String strBuffer = "";
  uint16_t i;

  if ((wifinderFlags.scanRssi || wifinderFlags.scanWifi) && wifiCounts >= 0)
  {
    if (wifinderFlags.scanRssi)
    {
      lastestRSSI = MIN_RSSI_VALUE;
      strBuffer = RSSI_EMPTY_VALUE;

      for (i = 0; i < wifiCounts; i++)
      {
        if (WiFi.SSID(i) == targetWifiSSID)
        {
          lastestRSSI = WiFi.RSSI(i);
          strBuffer = WiFi.RSSI(i);
          break;
        }
      }

      updateRSSIArc(lastestRSSI);
      lv_label_set_text(ui_RSSILabel, strBuffer.c_str());
      WiFi.scanDelete();
    }
    else if (wifinderFlags.scanWifi)
    {
      for (i = 0; i < wifiCounts; i++)
      {
        if (i > 0)
        {
          strBuffer += '\n';
        }
        strBuffer += WiFi.SSID(i);
      }

      lv_roller_set_options(ui_WifiSSIDListRoller, strBuffer.c_str(), LV_ROLLER_MODE_NORMAL);
    }

    WiFi.scanNetworks(true, false, false);
    fullUpdateScreen();
  }

  if (wifinderFlags.scanRssi)
  {
    emitAlarm(lastestRSSI);
  }

  lv_task_handler();
}

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  TouchPoint touchPos = touchScreen.read();
  if (touchPos.touched)
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchPos.xPos;
    data->point.y = touchPos.yPos;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

static void fullUpdateScreen()
{
  if (wifinderFlags.scanWifi)
  {
    lv_label_set_text(ui_ToggleScanLabel, STOP_WIFI_SCAN_TEXT);
  }
  else
  {
    lv_label_set_text(ui_ToggleScanLabel, START_WIFI_SCAN_TEXT);
  }

  if (wifinderFlags.scanRssi)
  {
    lv_obj_clear_flag(ui_WifiSSIDListRoller, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_state(ui_ToggleScanButton, LV_STATE_DISABLED);
    lv_label_set_text(ui_ToggleSelectWifiLabel, STOP_RSSI_SCAN_TEXT);
  }
  else
  {
    lv_obj_add_flag(ui_WifiSSIDListRoller, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_state(ui_ToggleScanButton, LV_STATE_DISABLED);
    lv_label_set_text(ui_ToggleSelectWifiLabel, START_RSSI_SCAN_TEXT);
  }

  uint16_t rollerOptionLength = getWifiOptionsLength();

  if (rollerOptionLength == 0)
  {
    lv_obj_add_state(ui_ToggleSelectWifiButton, LV_STATE_DISABLED);
  }
  else
  {
    lv_obj_clear_state(ui_ToggleSelectWifiButton, LV_STATE_DISABLED);
  }
}

void toggleWifiScan()
{
  if (wifinderFlags.scanWifi)
  {
    stopWifiScan();
  }
  else
  {
    beginWifiScan();
  }

  fullUpdateScreen();
}

static void beginWifiScan()
{
  wifinderFlags.scanWifi = true;
  WiFi.scanNetworks(true, false, false);

  updateRSSIArc(MIN_RSSI_VALUE);
  lv_roller_set_options(ui_WifiSSIDListRoller, "", LV_ROLLER_MODE_NORMAL);
}

static void stopWifiScan()
{
  wifinderFlags.scanWifi = false;
}

void toggleSelectWifi()
{
  if (wifinderFlags.scanRssi)
  {
    stopRssiScan();
  }
  else if (getWifiOptionsLength() > 0)
  {
    beginRssiScan();
  }

  fullUpdateScreen();
}

static void beginRssiScan()
{
  wifinderFlags.scanRssi = true;
  WiFi.scanNetworks(true, false, false);

  lastestRSSI = MIN_RSSI_VALUE;
  lastestAlarmTs = millis();

  lv_label_set_text(ui_RSSILabel, RSSI_EMPTY_VALUE);

  updateRSSIArc(MIN_RSSI_VALUE);

  lv_roller_get_selected_str(ui_WifiSSIDListRoller, targetWifiSSID, SSID_MAX_LENGTH);
}

static void stopRssiScan()
{
  wifinderFlags.scanRssi = false;
  digitalWrite(BUZZER_GPIO, LOW);
}

static uint16_t getWifiOptionsLength()
{
  if (strlen(lv_roller_get_options(ui_WifiSSIDListRoller)) == 0)
  {
    return 0;
  }
  return lv_roller_get_option_cnt(ui_WifiSSIDListRoller);
}

static lv_color_t getRssiColor(int32_t rssi)
{
  const lv_color_t redColor = lv_color_hex(0xcc3232);
  const lv_color_t yellowColor = lv_color_hex(0xe7b416);
  const lv_color_t greenColor = lv_color_hex(0x01735C);

  if (rssi >= -60)
  {
    return lv_color_mix(yellowColor, greenColor, map(rssi, -60, 0, 255, 0));
  }
  else
  {
    return lv_color_mix(redColor, yellowColor, map(rssi, -120, -60, 255, 0));
  }
}

static void updateRSSIArc(int32_t rssi)
{
  lv_arc_set_value(ui_RSSIArc, rssi);
  lv_color_t arcColor = getRssiColor(rssi);
  lv_obj_set_style_arc_color(ui_RSSIArc, arcColor, LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(ui_RSSIArc, arcColor, LV_PART_KNOB | LV_STATE_DEFAULT);
}

#define BEEP_TIME (250)
#define MIN_BEEP_DELAY_TIME (250)
#define MAX_BEEP_DELAY_TIME (2000)

static void emitAlarm(int32_t rssi)
{
  uint32_t totalAlarmTime = map(rssi, -120, 0, MAX_BEEP_DELAY_TIME, BEEP_TIME + MIN_BEEP_DELAY_TIME);

  uint32_t dt = millis() - lastestAlarmTs;

  if (dt > totalAlarmTime)
  {
    dt = 0;
    lastestAlarmTs = millis();
  }

  if (dt <= BEEP_TIME)
  {
    digitalWrite(BUZZER_GPIO, HIGH);
  }
  else
  {
    digitalWrite(BUZZER_GPIO, LOW);
  }
}