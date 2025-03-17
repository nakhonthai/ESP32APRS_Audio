//Simple sketch, no real benefit from DMA demenstrated here.
#include <Adafruit_GFX_Buffer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "Trident_Blue_Knob.h"
#include <T4_PXP.h>

typedef Adafruit_ST7789 display_t;
typedef Adafruit_GFX_Buffer<display_t> GFXBuffer_t;
GFXBuffer_t display_buffer = GFXBuffer_t(135, 240, display_t(&SPI, 35, 34, 33));
//Adafruit_ST7789 display_buffer = Adafruit_ST7789(&SPI, 35, 34, 33);

GFXcanvas16 srcBuf = GFXcanvas16(24,24);
GFXcanvas16 dstBuf = GFXcanvas16(64,64);

void setup() {
  display_buffer.init(135,240);
  display_buffer.setSPISpeed(60000000);
  display_buffer.setRotation(3);
  Serial.println(display_buffer.initDMA(display_buffer.DMA0) ? "DMA: On" : "DMA: Off");
  
//  printPXP();

  Serial.println("PXP_init!");
  PXP_init();
  printPXP();
  
  srcBuf.fillScreen(ST77XX_GREEN);

  PXP_overlay_buffer((void*)Trident_Blue_Knob.pixels, Trident_Blue_Knob.bytesPerPixel, Trident_Blue_Knob.width, Trident_Blue_Knob.height);
  PXP_overlay_format(PXP_RGB565, 1, true, 0xB0);
  PXP_overlay_color_key_low(Trident_Blue_Knob.colorKeyLow);
  PXP_overlay_color_key_high(Trident_Blue_Knob.colorKeyHigh);
  PXP_overlay_position(0, 0, (Trident_Blue_Knob.width) - 1, (Trident_Blue_Knob.height) - 1);

  PXP_input_buffer(srcBuf.getBuffer(), 2, srcBuf.height(), srcBuf.width());
  PXP_input_format(PXP_RGB565);
  PXP_input_position(27, 27, srcBuf.height() + 27, srcBuf.width() + 27);
  PXP_input_background_color(0x0000FF);
  
  PXP_output_buffer(dstBuf.getBuffer(), 2, dstBuf.height(), dstBuf.width());
  PXP_output_format(PXP_RGB565);
//  PXP_rotate(2);
//  PXP_flip_both(true);
}

void loop() {
  display_buffer.fillScreen(ST77XX_WHITE);
  srcBuf.fillScreen(ST77XX_GREEN);
  srcBuf.drawCircle(srcBuf.width()/2, srcBuf.height()/2, 4, 0xFFFF);

  uint32_t cycleStart, cycleEnd;
  Serial.println("PXP_process!");
  cycleStart = ARM_DWT_CYCCNT;
  PXP_process();
//  printPXP();
//  Serial.println("PXP_finish!");
  PXP_finish();
  cycleEnd = ARM_DWT_CYCCNT;
  Serial.printf("PXP finished in %lu cycles!\n", cycleEnd - cycleStart);
  
  printPXP();

  cycleStart = ARM_DWT_CYCCNT;
  display_buffer.drawRGBBitmap(80, 27, srcBuf.getBuffer(), srcBuf.width(), srcBuf.height());
  display_buffer.drawRGBBitmap(0, 0, dstBuf.getBuffer(), dstBuf.width(), dstBuf.height());
  cycleEnd = ARM_DWT_CYCCNT;
  Serial.printf("Draw RGB in %lu cycles!\n", cycleEnd - cycleStart);
  
  display_buffer.display();
  delay(500);
}

void printPXP(){
  Serial.printf("CTLR: %lx \n", PXP_CTRL);
  Serial.printf("STAT: %lx \n", PXP_STAT);
  Serial.printf("OUT_CTLR: %lx \n", PXP_OUT_CTRL);
//  Serial.printf("OUT_BUF: %lx   dstBuf: %lx\n", PXP_OUT_BUF, (uint32_t)display_buffer.getBuffer());
  Serial.printf("OUT_BUF: %lx   dstBuf: %lx\n", PXP_OUT_BUF, (uint32_t)dstBuf.getBuffer());
  Serial.printf("OUT_PITCH: %lu \n", PXP_OUT_PITCH);
  Serial.printf("OUT_LRC: %lx \n", PXP_OUT_LRC);
  Serial.printf("PS_CTLR: %lx \n", PXP_PS_CTRL);
  Serial.printf("PS_BUF: %lx   srcBuf: %lx\n", PXP_PS_BUF, (uint32_t)srcBuf.getBuffer());
  Serial.printf("PS_PITCH: %lu \n", PXP_PS_PITCH);
  Serial.printf("PS_BG: %lx \n", PXP_PS_BACKGROUND_0);
  Serial.printf("PS_SCALE: %lx \n", PXP_PS_SCALE);
  Serial.printf("PS_OFFSET: %lx \n", PXP_PS_OFFSET);
  Serial.printf("PS_ULC: %lx \n", PXP_OUT_PS_ULC);
  Serial.printf("PS_LRC: %lx \n", PXP_OUT_PS_LRC);
  Serial.printf("AS_CTLR: %lx \n", PXP_AS_CTRL);
  Serial.printf("AS_BUF: %lx   bitmap: %lx\n", PXP_AS_BUF, (uint32_t)Trident_Blue_Knob.pixels);
  Serial.printf("AS_PITCH: %lu \n", PXP_AS_PITCH);
  Serial.printf("AS_ULC: %lx \n", PXP_OUT_AS_ULC);
  Serial.printf("AS_LRC: %lx \n", PXP_OUT_AS_LRC);
  Serial.printf("NEXT: %lx \n", PXP_NEXT);

  Serial.printf("\n\n");
  Serial.send_now();
}
