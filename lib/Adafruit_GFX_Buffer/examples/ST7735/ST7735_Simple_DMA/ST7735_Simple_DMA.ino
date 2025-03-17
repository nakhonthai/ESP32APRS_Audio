//Simple sketch, no real benefit from DMA demenstrated here.
#include <Adafruit_GFX_Buffer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

typedef Adafruit_ST7735 display_t;
typedef Adafruit_GFX_Buffer<display_t> GFXBuffer_t;
GFXBuffer_t display_buffer = GFXBuffer_t(80, 160, display_t(&SPI, 35, 34, 33));

void setup() {
//  display_buffer.initS(60000000); //Unofficial modification for ST7735S displays
  display_buffer.initR(INITR_MINI160x80);
  display_buffer.setSPISpeed(60000000);
  display_buffer.setRotation(1);
  Serial.println(display_buffer.initDMA(display_buffer.DMA0) ? "DMA: On" : "DMA: Off");
}

void loop() {
  display_buffer.fillScreen(ST77XX_BLACK);
  display_buffer.display();
  delay(500);
  display_buffer.fillScreen(ST77XX_WHITE);
  display_buffer.display();
  delay(500);
}
