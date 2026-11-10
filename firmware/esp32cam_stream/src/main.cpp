// ESP32-CAM (AI-Thinker) — streams JPEG frames over Serial/USB instead of
// WiFi. Pairs with SerialCamera in python/pick_and_place.py, which reads this
// same USB-TTL (e.g. CH340) cable and parses frames by their JPEG SOI/EOI
// markers (0xFFD8 ... 0xFFD9), so no length-prefix framing is needed here —
// just write raw JPEGs back to back.
//
// Board: "AI Thinker ESP32-CAM" in Arduino IDE board manager.
// Baud must match ESP32_SERIAL_BAUD in app.py (default 921600).

#include <Arduino.h>
#include "esp_camera.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Measured on this CH340: 460800 gives no bad frames; 576000 and up corrupt
// frames even with the gap after each frame. Set with STREAM_BAUD in
// platformio.ini; must match camera.baud on the Pi.
static const uint32_t SERIAL_BAUD = STREAM_BAUD;
static const uint32_t FRAME_GAP_MS = 2;

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setDebugOutput(false); // keep the serial line clean of ESP-IDF logs

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST; // send the newest frame, not one queued behind the UART

  // VGA keeps each frame small enough for decent FPS over Serial at
  // SERIAL_BAUD. Raise jpeg_quality (smaller file) if frames are still
  // too large for a smooth rate; lower it for better image quality.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = STREAM_QUALITY;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Printed once, before any JPEG bytes are ever sent, so it can't
    // corrupt the frame stream the Pi parses. Watch this in the Arduino
    // Serial Monitor (921600 baud) — NOT while app.py has the port open,
    // only one process can hold /dev/ttyUSB0 at a time.
    Serial.printf("CAM_INIT_FAIL err=0x%x\n", err);
    while (true) {
      delay(1000);
    }
  }

  Serial.println("CAM_OK");
}

uint32_t last_heartbeat = 0;

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    Serial.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
    Serial.flush();
    delay(FRAME_GAP_MS);
  } else if (millis() - last_heartbeat > 2000) {
    // fb_get() returned nothing — sensor/DMA issue. This text lands between
    // JPEGs, never inside one, so it won't corrupt the Pi-side parser.
    Serial.println("CAM_FB_NULL");
    last_heartbeat = millis();
  }
}
