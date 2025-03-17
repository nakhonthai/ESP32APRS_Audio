#include <Adafruit_GFX_Buffer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

//BuyDisplay ST7735S display is 80x160 before rotating so set the buffer accordingly
typedef Adafruit_ST7735 display_t;
typedef Adafruit_GFX_Buffer<display_t> GFXBuffer;
GFXBuffer display_buffer0 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 35, 34, 33));
GFXBuffer display_buffer1 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 36, 34, -1));
GFXBuffer display_buffer2 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 37, 34, -1));
GFXBuffer* display_buffer[] = {
  &display_buffer0,
  &display_buffer1,
  &display_buffer2,
};
uint8_t number_of_displays = sizeof(display_buffer) / sizeof(*display_buffer);

void setup() {
  for(uint8_t i = 0; i < number_of_displays; i++){
    display_buffer[i]->initR(INITR_MINI160x80);
    display_buffer[i]->setSPISpeed(60000000);
    display_buffer[i]->setRotation(3);
  }
}

void loop() {
  for(uint8_t i = 0; i < number_of_displays; i++){
    rgb_test(display_buffer[i]);
  }
}

void drawStringCenterBoth(GFXcanvas16 *canvas, uint16_t x, uint16_t y, const char *text, uint16_t color = 0xFFFF, uint16_t bg_color = 0xFFFF, uint16_t size = 1){
  canvas->setTextSize(size);
  canvas->setTextColor(color, bg_color);
  uint16_t width, height;
  int16_t x1, y1;
  canvas->getTextBounds(text, 0, 0, &x1, &y1, &width, &height);
  canvas->setCursor(x-(width/2), y-(height/2));
  canvas->write(text);
}

void rgb_test(GFXBuffer* display_pointer){
  draw_test_screen(display_pointer, color565(0xFF,0,0));
  display_pointer->display();
  delay(500);
  draw_test_screen(display_pointer, color565(0,0xFF,0));
  display_pointer->display();
  delay(500);
  draw_test_screen(display_pointer, color565(0,0,0xFF));
  display_pointer->display();
  delay(500);
}

void draw_test_screen(GFXcanvas16 *canvas, uint16_t bg_color){
  canvas->fillScreen(bg_color);
  canvas->drawCircle(canvas->width()/2, canvas->height()/2, 20, 0xFFFF);
  drawStringCenterBoth(canvas, canvas->width()/2, canvas->height()/2, "This is a test!");
}
