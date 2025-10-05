
/*
  Version 0.5
  PressureDisplay.ino
  Display simulated pressure readings on T-Display-S3 with a bar graph and min/max values.
  Press the user button (IO14) to reset min/max.
*/

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "pin_config.h"
// Include converted image data (8bpp with 256-entry RGB palette)
#include "fh_logo.c"

#define REFRESH_MS 40  // refresh interval in milliseconds

#define PRESSURE_CLAMPED 5
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

// Draw a horizontal bar at the bottom of the screen representing pressure
// Minimal-update implementation: border drawn once in setup(), this updates
// only the inner area to avoid flicker.
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
void setup(void)
{
  //Serial.begin(115200);

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  // Initialize user button IO14 (internal pullup, active LOW)
  pinMode(PIN_BTN_IO14, INPUT_PULLUP);

  tft.init();
  tft.setRotation(3); // tole bomo potem dali na 1
  tft.fillScreen(TFT_BLACK);

  // Draw pre-converted fh_logo in the upper-left corner
  // fh_logo.c layout: 8 byte header, 256*3 bytes RGB palette, then width*height bytes of 8bpp indices
  {
    // Read width/height from header (little endian)
    uint16_t imgW = (uint16_t)gImage_fh_logo[2] | ((uint16_t)gImage_fh_logo[3] << 8);
    uint16_t imgH = (uint16_t)gImage_fh_logo[4] | ((uint16_t)gImage_fh_logo[5] << 8);
    const uint8_t *palette = &gImage_fh_logo[8];
    const uint8_t *pixels = &gImage_fh_logo[8 + 256 * 3];

    // Build 16-bit colour map (RGB888 -> RGB565) in RAM
    static uint16_t cmap[256];
    for (int i = 0; i < 256; ++i) {
      uint8_t r = pgm_read_byte(&palette[i * 3 + 0]);
      uint8_t g = pgm_read_byte(&palette[i * 3 + 1]);
      uint8_t b = pgm_read_byte(&palette[i * 3 + 2]);
      uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
      cmap[i] = c;
    }

    // Push image by converting each row to 16-bit and sending it
    // Some examples set swap bytes when using data from converters
    tft.setSwapBytes(true);
    uint16_t lineBuf[imgW];
    const int imgX = 0;      // left edge
    const int imgY = 15;     // shift image down by 20 pixels
    for (int row = 0; row < imgH; ++row) {
      const uint8_t *prow = pixels + row * imgW;
      for (int col = 0; col < imgW; ++col) {
        uint8_t idx = pgm_read_byte(prow + col);
        lineBuf[col] = cmap[idx];
      }
      // push single line at offset Y = row + imgY
      tft.pushImage(imgX, row + imgY, imgW, 1, lineBuf);
    }
    tft.setSwapBytes(false);
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
    pressure = pressure + 1.1;
    if (pressure > 200.00) pressure = -10.00;
    // ===============================================================================
    if (pressure > max_pressure) max_pressure = pressure;
    if (pressure < min_pressure) min_pressure = pressure;
    
    // Draw pressure bar at bottom of screen (0..175 mapped to full width)
    drawPressureBar(pressure);

    // --- Button handling: IO14 resets min/max on press-release ---
    int rawBtn = digitalRead(PIN_BTN_IO14);
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
    
    xpos = 140; ypos = 100;
    if (pressure < PRESSURE_CLAMPED) {
      tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
      tft.drawString("CLAMPED        ", xpos, ypos, 4);
    } else if (pressure > PRESSURE_OVER) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("OVERLOAD    ", xpos, ypos, 4);
    } else if (pressure >= PRESSURE_UNCLAMPED && pressure <= PRESSURE_OVER) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("UNCLAMPED    ", xpos, ypos, 4);
    } else {
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      tft.drawString("                             ", xpos, ypos, 4);
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
