
/*
  An example showing rainbow colours on a 1.8" TFT LCD screen
  and to show a basic example of font use.

  Make sure all the display driver and pin connections are correct by
  editing the User_Setup.h file in the TFT_eSPI library folder.

  Note that yield() or delay(0) must be called in long duration for/while
  loops to stop the ESP8266 watchdog triggering.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "pin_config.h"
// Include converted image data (8bpp with 256-entry RGB palette)
#include "fh_logo.c"



TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

unsigned long targetTime = 0;
byte red = 31;
byte green = 0;
byte blue = 0;
byte state = 0;
unsigned int colour = red << 11;
uint32_t runing = 0;
float randPressure = 1; // Declare randPressure as a global variable

void setup(void)
{
  
 
  Serial.begin(115200);

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

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
    for (int row = 0; row < imgH; ++row) {
      const uint8_t *prow = pixels + row * imgW;
      for (int col = 0; col < imgW; ++col) {
        uint8_t idx = pgm_read_byte(prow + col);
        lineBuf[col] = cmap[idx];
      }
      // push single line
      tft.pushImage(0, row, imgW, 1, lineBuf);
    }
    tft.setSwapBytes(false);
  }

  targetTime = millis() + 1000;
}

void loop()
{

  if (millis() > runing) {
    Serial.print("Current running ");
    Serial.print(millis());
    Serial.println(" millis");
    runing = millis() + 5000;
  }
  if (targetTime < millis()) {
    targetTime = millis() + 50;


    // The standard ADAFruit font still works as before
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    // tft.setCursor (12, 5);
    // tft.print("Original ADAfruit font!");

    // The new larger fonts do not use the .setCursor call, coords are embedded
    // tft.setTextColor(TFT_BLACK, TFT_BLACK); // Do not plot the background colour

    // Overlay the black text on top of the rainbow plot (the advantage of not drawing the backgorund colour!)
    // tft.drawCentreString("Font size 2", 80, 14, 2); // Draw text centre at position 80, 12 using font 2

    //tft.drawCentreString("Font size 2",81,12,2); // Draw text centre at position 80, 12 using font 2

    // tft.drawCentreString("bar", 80, 30, 4); // Draw text centre at position 80, 24 using font 4

    // tft.drawCentreString("12345678", 80, 54, 4); // Draw text centre at position 80, 24 using font 6

    // tft.drawCentreString("12.34 is in font size 6", 80, 92, 2); // Draw text centre at position 80, 90 using font 2

    // Note the x position is the top left of the font!
  
  
  //float randPressure = 100.00 + (float)random(0,200)/100; // random integer [0..200]
  randPressure = randPressure + 0.1;
  if (randPressure > 200.00) randPressure = 0.00;

  // draw a random integer between 0 and 200
  int xpos = 50;      // x position
  int ypos = 90;      // y position
  int font = 6;       // font number only 2,4,6,7 valid. Font 6 contains digits

  // Compute the bounding box for the numeric text and unit so we can clear it
  // Use fontHeight() to get line height for the fonts used
  int16_t numH = tft.fontHeight(font);
  // Estimate number width: drawNumber will return the delta (width) so we can compute after drawing
  // Clear area behind previous number+unit: choose a conservative width (e.g., 120 px)
  int clearW = 135;
  int clearH = max(numH, tft.fontHeight(4));
  //tft.fillRect(xpos - 2, ypos - 2, clearW, clearH + 4, TFT_BLACK);
  xpos += tft.drawFloat(randPressure, 1, xpos, ypos, font); // Draw rounded number and return new xpos delta for next print position
  tft.drawString(" bar", 180, ypos+18, 4); // Continue printing from new x position
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
