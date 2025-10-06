
/*
  Version 0.5
  PressureDisplay.ino
  Display simulated pressure readings on T-Display-S3 with a bar graph and min/max values.
  Press the user button (IO14) to reset min/max.
*/

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <Wire.h>
#include "pin_config.h"
// Include converted image data (8bpp with 256-entry RGB palette)
#include "fh_logo.c"
// splash logo (8bpp converter output) - add a file named splash_logo.c to the sketch folder
#include "splash_logo.c"

#define REFRESH_MS 40  // refresh interval in milliseconds

#define PRESSURE_CLAMPED 7
#define PRESSURE_UNCLAMPED 140
#define PRESSURE_OVER 155

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

unsigned long targetTime = 0;
byte red = 31;
byte green = 0;
byte blue = 0;
byte state = 0;
unsigned int colour = red << 11;
uint32_t runing = 0;
float pressure = -10; // Declare randPressure as a global variable
float max_pressure = 0;
float min_pressure = 999;
// Button (IO14) used to reset min/max
const uint8_t PIN_BTN_IO14 = 14; // user button IO14 (active LOW)
// BOOT button (often labelled BUTTON_1 in pin_config)
const uint8_t PIN_BTN_BOOT = PIN_BUTTON_1; // usually 0 on the board
// debounce state
bool btnLastState = HIGH;
unsigned long btnLastChange = 0;
unsigned long btnPressedAt = 0;
const unsigned long DEBOUNCE_MS = 30;
const unsigned long LONGPRESS_MS = 1000; // not used, but available

// Bar graph settings
const int BAR_HEIGHT = 30;     // lowest 30 pixels
const float BAR_MAX = 175.0;   // bar graph max mapping
const int barY = 170 - BAR_HEIGHT; // Y position of the bar

// --- ADS1115 I2C configuration and helper functions -----------------------
// I2C pins for ESP32-S3
const int I2C_SDA_PIN = 18;
const int I2C_SCL_PIN = 17;
// ADS1115 default I2C address
const uint8_t ADS1115_ADDR = 0x48;
// ADS1115 registers
const uint8_t ADS_REG_CONVERSION = 0x00;
const uint8_t ADS_REG_CONFIG = 0x01;
// Using PGA = +/-4.096V -> LSB = 125uV
const float ADS1115_LSB = 0.000125f; // volts per bit for +/-4.096V

// Resistor divider: sensor (0-10V) -> divider -> ADS1115 AIN0
// Top resistor (from sensor output to ADS input) = 75k, bottom resistor (to GND) = 33k
const float R_TOP = 75000.0f; // ohms
const float R_BOTTOM = 33000.0f; // ohms

// Sensor mapping: 0..10V -> 0..200 bar
const float SENSOR_V_MAX = 10.0f; // volts at 200 bar
const float SENSOR_P_MAX = 200.0f; // bar

// Write a 16-bit register to ADS1115 (MSB first)
void ads1115WriteRegister(uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

// Read a 16-bit register from ADS1115 (MSB first)
int16_t ads1115ReadRegister(uint8_t reg)
{
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    // NACK or bus error
    return INT16_MIN;
  }
  Wire.requestFrom((int)ADS1115_ADDR, 2);
  if (Wire.available() < 2) return INT16_MIN;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// Perform a single-shot conversion on AIN0 and return measured voltage in volts.
// Returns NAN on error.
float readADS1115Voltage()
{
  // Build config: start single conversion, MUX = AIN0 single-ended (100), PGA = +/-4.096V (001),
  // single-shot mode, 128SPS (100), comparator disabled (11)
  uint16_t config = 0;
  config |= (1 << 15);            // OS = 1: start single conversion
  config |= (0b100 << 12);        // MUX = 100 -> AIN0 relative to GND
  config |= (0b001 << 9);         // PGA = 001 -> +/-4.096V
  config |= (1 << 8);             // MODE = 1 -> single-shot
  config |= (0b100 << 5);         // DR = 100 -> 128 SPS
  config |= 0b11;                 // COMP_QUE = 11 -> disable comparator

  ads1115WriteRegister(ADS_REG_CONFIG, config);

  // Wait for conversion to complete. At 128SPS conversion ~7.8ms, so poll with timeout
  const unsigned long timeout = 50; // ms
  unsigned long start = millis();
  while (millis() - start < timeout) {
    int16_t cfg = ads1115ReadRegister(ADS_REG_CONFIG);
    if (cfg == INT16_MIN) break; // bus error
    if (cfg & (1 << 15)) {
      // OS bit set -> conversion complete
      int16_t raw = ads1115ReadRegister(ADS_REG_CONVERSION);
      if (raw == INT16_MIN) return NAN;
      // For single-ended reads raw is a positive number (signed 16-bit)
      float voltage = raw * ADS1115_LSB;
      return voltage;
    }
    delay(2);
  }

  // Timeout or error: try a final read of conversion register
  int16_t raw = ads1115ReadRegister(ADS_REG_CONVERSION);
  if (raw == INT16_MIN) return NAN;
  return raw * ADS1115_LSB;
}

// Draw a horizontal bar at the bottom of the screen representing pressure
// Minimal-update implementation: border drawn once in setup(), this updates
// only the inner area to avoid flicker.
bool blink()
{
  if (millis() % 300 < 150)
    return true;
  else
    return false;
}

void drawPressureBar(float p)
{
  int screenW = tft.width();
  int screenH = tft.height();
  int y = screenH - BAR_HEIGHT;

  // inner area (leave 1px border intact)
  int ix = 1;
  int iy = y + 1;
  int iW = screenW - 2;
  int iH = BAR_HEIGHT - 2;

  // Clamp pressure to 0..BAR_MAX
  float pc = p;
  if (pc < 0.0) pc = 0.0;
  if (pc > BAR_MAX) pc = BAR_MAX;

  // Compute inner width in pixels
  int innerBarW = (int)((pc / BAR_MAX) * (float)iW + 0.5);

  static int prevInnerW = -1;
  uint32_t bar_color = TFT_SKYBLUE;
  if (prevInnerW < 0) {
    // first-time init: clear inner background and draw initial bar
    tft.fillRect(ix, iy, iW, iH-1, TFT_BLACK);
    if (innerBarW > 0) tft.fillRect(ix, iy, innerBarW, iH-1, bar_color);
  } else if (innerBarW > prevInnerW) {
    // bar grew: draw only the newly exposed area
    tft.fillRect(ix + prevInnerW, iy, innerBarW - prevInnerW, iH-1, bar_color);
  } else if (innerBarW < prevInnerW) {
    // bar shrank: erase only the removed area
    tft.fillRect(ix + innerBarW, iy, prevInnerW - innerBarW, iH-1, TFT_BLACK);
  }

  prevInnerW = innerBarW;
}

// Helper to draw a 16-bit RGB565 image stored as little-endian words after an 8-byte header
void draw16bppImage(const uint8_t *img, int destX, int destY)
{
  uint16_t imgW = (uint16_t)img[2] | ((uint16_t)img[3] << 8);
  uint16_t imgH = (uint16_t)img[4] | ((uint16_t)img[5] << 8);
  const uint8_t *pixels = &img[8]; // expected layout: 2 bytes per pixel, little-endian (lo,hi)

  tft.setSwapBytes(true); // ensure correct byte order for pushImage
  // allocate line buffer on stack; if width is large consider static allocation
  uint16_t lineBuf[imgW];
  for (int row = 0; row < imgH; ++row) {
    const uint8_t *prow = pixels + row * imgW * 2;
    for (int col = 0; col < imgW; ++col) {
      uint16_t lo = pgm_read_byte(prow + col * 2);
      uint16_t hi = pgm_read_byte(prow + col * 2 + 1);
      lineBuf[col] = (hi << 8) | lo; // reconstruct little-endian 16-bit pixel
    }
    tft.pushImage(destX, destY + row, imgW, 1, lineBuf);
  }
  tft.setSwapBytes(false);
}

// Show splash screen using splash_logo.c for 2 seconds
void showSplash()
{
  tft.fillScreen(TFT_BLACK);
  // center the splash image
  uint16_t w = (uint16_t)gImage_splash_logo[2] | ((uint16_t)gImage_splash_logo[3] << 8);
  uint16_t h = (uint16_t)gImage_splash_logo[4] | ((uint16_t)gImage_splash_logo[5] << 8);
  int x = (tft.width() - w) / 2;
  int y = (tft.height() - h) / 2 - 20; // lift a bit to make room for text
  draw16bppImage((const uint8_t *)gImage_splash_logo, x, y);

  // draw text below image
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.drawCentreString("Hexapod pressure indicator", tft.width() / 2, y + h + 6, 2);
  tft.drawCentreString("FW: v1.0", tft.width() / 2, y + h + 22, 2);

  delay(2000);
  // clear splash area (simple full-screen clear; setup will redraw UI)
  tft.fillScreen(TFT_BLACK);
}
void setup(void)
{
  //Serial.begin(115200);

  // Initialize I2C for ADS1115 (SDA=18, SCL=17)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  // Initialize user button IO14 (internal pullup, active LOW)
  pinMode(PIN_BTN_IO14, INPUT_PULLUP);
  // initialize BOOT button the same way
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);

  tft.init();
  tft.setRotation(3); // tole bomo potem dali na 1

  showSplash();

  tft.fillScreen(TFT_BLACK);

  // Draw pre-converted fh_logo in the upper-left corner using helper
  // fh_logo.c layout: 8 byte header, 256*3 bytes RGB palette, then width*height bytes of 8bpp indices
  {
    const int imgX = 0;      // left edge
    const int imgY = 15;     // shift image down by 20 pixels
    draw16bppImage((const uint8_t *)gImage_fh_logo, imgX, imgY);
  }

    // Draw static border for the bar area once (avoid redrawing border every frame)
    {
      int screenW = tft.width();
      int screenH = tft.height();
      int by = screenH - BAR_HEIGHT;
      tft.drawRect(0, by, screenW, BAR_HEIGHT, TFT_LIGHTGREY);
      // DRAW 150 bar mark
      int x150 = (int)((150.0 / BAR_MAX) * (float)(screenW - 2) + 0.5) + 1;
      tft.fillTriangle(x150 - 3, by - 15, x150 + 3, by - 15, x150, by-1, TFT_WHITE);
    }

    targetTime = millis() + 100;

    // Optional: perform a single ADS1115 read to ensure communication (uncomment to use)
    // float v = readADS1115Voltage();
    // if (!isnan(v)) {
    //   Serial.print("ADS1115 A0 voltage: "); Serial.println(v, 6);
    // }
}

void loop()
{

  // if (millis() > runing) {
  //   Serial.print("Current running ");
  //   Serial.print(millis());
  //   Serial.println(" millis");
  //   runing = millis() + 5000;
  // }
  if (targetTime < millis()) {
    targetTime = millis() + REFRESH_MS;
    // ===============================================================================
    // pressure = 0.00 + (float)random(0,2000)/10; // random integer [0..200]
    
    // pressure = pressure + 0.6;
    // if (pressure > 200.00) pressure = -10.00;
    
    // // ===============================================================================
    
    // Read divider output voltage from ADS1115 and convert to sensor input voltage
    float v_div = readADS1115Voltage(); // measured at divider bottom (to GND)
    if (isnan(v_div)) {
      pressure = -999.0; // error reading
    } else {
      // Divider relationship: v_div = v_in * (R_BOTTOM / (R_TOP + R_BOTTOM))
      float v_in = v_div * ((R_TOP + R_BOTTOM) / R_BOTTOM);
      // Map 0..SENSOR_V_MAX -> 0..SENSOR_P_MAX
      pressure = (v_in / SENSOR_V_MAX) * SENSOR_P_MAX;
    }
    
    if (pressure > max_pressure) max_pressure = pressure;
    if (pressure < min_pressure) min_pressure = pressure;
    
    // Draw pressure bar at bottom of screen (0..175 mapped to full width)
    drawPressureBar(pressure);

    // --- Button handling: IO14 resets min/max on press-release ---
    // read both buttons (BOOT and IO14); consider them equivalent
    int rawBtnA = digitalRead(PIN_BTN_IO14);
    int rawBtnB = digitalRead(PIN_BTN_BOOT);
    // collapse to a single logical input (LOW when either is pressed)
    int rawBtn = (rawBtnA == LOW || rawBtnB == LOW) ? LOW : HIGH;
    if (rawBtn != btnLastState) {
      btnLastChange = millis();
      btnLastState = rawBtn;
    }
    if (millis() - btnLastChange > DEBOUNCE_MS) {
      // stable state
      static bool btnStableLast = HIGH;
      if (rawBtn != btnStableLast) {
        btnStableLast = rawBtn;
        if (btnStableLast == LOW) {
          // pressed
          btnPressedAt = millis();
        } else {
          // released -> treat as a short press (reset min/max)
          // Reset min and max values
          max_pressure = 0;
          min_pressure = 999;
          // Provide visual feedback: quickly flash a message
          //tft.fillRect(10, 140, 150, 24, TFT_BLACK);
          //tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          //tft.drawString("Min/Max reset", 12, 140, 2);
          // short delay so user sees feedback (non-blocking-ish)
          // We'll let the normal loop timing clear/update the display next frame
        }
      }
    }

    int xpos = 75;      // x position
    int ypos = 10;      // y position
    int font = 8;       // font number only 2,4,6,7 valid. Font 6 contains digits


    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    
    xpos += tft.drawFloat(pressure, 1, xpos, ypos, font); // Draw rounded number and return new xpos delta for next print position
    xpos += tft.drawString("    ",  xpos, ypos, font); // Draw rounded number and return new xpos delta for next print position

    xpos = 10; ypos = 95;
    xpos += tft.drawString("Max: ",  xpos, ypos, 2);
    xpos += tft.drawFloat(max_pressure, 1, xpos, ypos, 2); // Draw rounded number and return new xpos delta for next print position
    xpos += tft.drawString(" bar   ",  xpos, ypos, 2);
    xpos = 10; ypos = 115;
    xpos += tft.drawString("Min: ",  xpos, ypos, 2);
    xpos += tft.drawFloat(min_pressure, 1, xpos, ypos, 2); // Draw rounded number and return new xpos delta for next print position
    xpos += tft.drawString(" bar   ",  xpos, ypos, 2);
    
    xpos = 130; ypos = 100;
    if (pressure < PRESSURE_CLAMPED) {
      tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
      tft.drawString("CLAMPED          ", xpos, ypos, 4);
    } else if (pressure > PRESSURE_OVER && blink()) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("OVERLOAD        ", xpos, ypos, 4);
    } else if (pressure >= PRESSURE_UNCLAMPED && pressure <= PRESSURE_OVER && blink()) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("UNCLAMPED    " , xpos, ypos, 4);
    } else {
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      tft.drawString("                                  ", xpos, ypos, 4);
    }
  }
}


// TFT Pin check
#if PIN_LCD_WR  != TFT_WR || \
  PIN_LCD_RD  != TFT_RD || \
  PIN_LCD_CS    != TFT_CS   || \
  PIN_LCD_DC    != TFT_DC   || \
  PIN_LCD_RES   != TFT_RST  || \
  PIN_LCD_D0   != TFT_D0  || \
  PIN_LCD_D1   != TFT_D1  || \
  PIN_LCD_D2   != TFT_D2  || \
  PIN_LCD_D3   != TFT_D3  || \
  PIN_LCD_D4   != TFT_D4  || \
  PIN_LCD_D5   != TFT_D5  || \
  PIN_LCD_D6   != TFT_D6  || \
  PIN_LCD_D7   != TFT_D7  || \
  PIN_LCD_BL   != TFT_BL  || \
  TFT_BACKLIGHT_ON   != HIGH  || \
  170   != TFT_WIDTH  || \
  320   != TFT_HEIGHT
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
#error  "The current version is not supported for the time being, please use a version below Arduino ESP32 3.0"
#endif
