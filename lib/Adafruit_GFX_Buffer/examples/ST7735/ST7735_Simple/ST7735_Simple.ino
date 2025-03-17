#include <Adafruit_GFX_Buffer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

typedef Adafruit_ST7735 display_t;
typedef Adafruit_GFX_Buffer<display_t> GFXBuffer_t;
GFXBuffer_t display_buffer = GFXBuffer_t(80, 160, display_t(&SPI, 35, 34, 33));

void setup() {
  display_buffer.initR(INITR_MINI160x80);
  display_buffer.setRotation(1);
}

void loop() {
  display_buffer.fillScreen(ST77XX_BLACK);
  display_buffer.display();
  delay(500);
  display_buffer.fillScreen(ST77XX_WHITE);
  display_buffer.display();
  delay(500);
}
