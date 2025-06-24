#include "esp_camera.h"
#include "driver/gpio.h" // For LED control

// Include TFT_eSPI library
#include <TFT_eSPI.h> // Include the TFT_eSPI library

// --- TFT Display Setup ---
TFT_eSPI tft = TFT_eSPI(); // Create a TFT_eSPI object

// --- Camera Model Definition ---
// UNCOMMENT this line for ESP32-S3-EYE. Comment out all others.
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h" // This file contains the pin definitions for the selected camera model

// --- Other Definitions ---
// Onboard LED connected to GPIO 21 on ESP32-S3-EYE
#define LED_PIN GPIO_NUM_21

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\nStarting Camera to LCD Stream...");

  // --- Initialize LED (Onboard Indicator) ---
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0); // Turn off LED initially (assuming active LOW)

  // --- Initialize TFT Display ---
  tft.init();
  tft.setRotation(0); // Adjust rotation if needed (0, 1, 2, 3)
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.println("Initializing Camera...");

  // Turn on backlight (GPIO 13 is TFT_BL, set HIGH for ON)
  gpio_reset_pin(GPIO_NUM_13);
  gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_13, 1); // HIGH for backlight ON

  // --- Camera Configuration ---
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
  config.xclk_freq_hz = 20000000; // 20MHz clock for the camera
  config.pixel_format = PIXFORMAT_RGB565; // Output RGB565 for direct display
  config.grab_mode = CAMERA_GRAB_LATEST; // Grab latest frame
  config.fb_location = CAMERA_FB_IN_PSRAM; // Use PSRAM for frame buffer
  config.frame_size = FRAMESIZE_QVGA; // Start with QVGA (320x240) for compatibility with 240x240 screen
  config.jpeg_quality = 12; // Not used for RGB565, but good to have a default
  config.fb_count = 1; // One frame buffer is often enough for display

  // If PSRAM is found, make sure frame_size is suitable
  if (psramFound()) {
    Serial.println("PSRAM found. Using PSRAM for frame buffer.");
    config.fb_count = 2; // Can use 2 buffers for smoother capture
    // Set frame_size to QVGA (320x240) which is commonly displayed on 240x240 screens (will crop/scale)
    config.frame_size = FRAMESIZE_QVGA;
  } else {
    Serial.println("No PSRAM found. Limiting frame size to VGA and DRAM.");
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.frame_size = FRAMESIZE_VGA; // Or even smaller if DRAM is limited
  }

  // Camera Initialization
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    tft.fillScreen(TFT_RED);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_WHITE);
    tft.printf("Camera init failed! 0x%x", err);
    while (true); // Halt if camera fails
  }

  sensor_t *s = esp_camera_sensor_get();
  // Initial sensor settings if needed (OV2640 specific)
  if (s->id.PID == OV2640_PID) {
    // You can adjust these for your environment
    s->set_vflip(s, 1);       // Flip vertically (often needed for orientation)
    // s->set_hmirror(s, 1);    // Mirror horizontally
    // s->set_brightness(s, 0); // -2 to 2
    // s->set_contrast(s, 0);   // -2 to 2
    // s->set_saturation(s, 0); // -2 to 2
  }

  // Set the camera frame size specifically to match or be larger than the display
  // For a 240x240 display, a 240x240, 320x240 (QVGA) or 320x320 resolution is good.
  // QVGA (320x240) is processed to fit the 240x240 screen.
  s->set_framesize(s, FRAMESIZE_QVGA);

  Serial.println("Camera Initialized. Starting stream to LCD.");
  tft.fillScreen(TFT_BLACK); // Clear screen after init message
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get(); // Get a camera frame buffer
  if (!fb) {
    Serial.println("Camera capture failed");
    return; // Skip this loop if capture fails
  }

  // Check if the pixel format is RGB565 (which we requested)
  if (fb->format == PIXFORMAT_RGB565) {
    // Push the image to the TFT display.
    // Assumes 240x240 screen. If QVGA (320x240) is captured:
    // It will be centered or cropped depending on how pushImage handles it.
    // For QVGA (320x240) to 240x240:
    // We'll crop 40 pixels from each side of the width (320 - 240 = 80; 80/2 = 40)
    // So the image starts at (40, 0) and is 240x240.
    int x_offset = (fb->width - tft.width()) / 2;
    int y_offset = (fb->height - tft.height()) / 2;

    if (x_offset < 0 || y_offset < 0) { // If screen is larger than capture
      x_offset = 0; y_offset = 0;
    }

    tft.pushImage(0, 0, tft.width(), tft.height(),
                  (uint16_t*)fb->buf + (y_offset * fb->width + x_offset));

  } else {
    // If you need to handle other formats (e.g., JPEG, convert to RGB565 first)
    Serial.printf("Unsupported pixel format: %d\n", fb->format);
  }

  esp_camera_fb_return(fb); // Return the frame buffer to the camera for reuse

  // Small delay to prevent watchdog timer issues and allow other tasks to run
  delay(1);
}