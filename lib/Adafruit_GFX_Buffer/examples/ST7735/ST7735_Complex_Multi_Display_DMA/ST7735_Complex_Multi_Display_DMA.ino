//More complex sketch that is frame rate locked to show the benefits of DMA
#include <Adafruit_GFX_Buffer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

//Change these if needed
uint8_t displayNumber = 12;  //Number of displays to update, 12 are setup below
uint8_t targetFPS = 10;     //FPS to target, can be changed from serial monitor

//Don't change
elapsedMillis oneSecond;
elapsedMillis internalRefresh;
uint32_t looped = 0, internal = 0, fps = 0, fps_actual = 0;
elapsedMicros screenRefresh;
uint8_t displayToUpdate = 0;

uint16_t solidColor = 0;

const uint16_t canvasWidth = 80;
const uint16_t canvasHeight = 160;

typedef Adafruit_ST7735 display_t;
typedef Adafruit_GFX_Buffer<display_t> GFXBuffer;
GFXBuffer display0 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 35, 34, 33));
GFXBuffer display1 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 36, 34, -1));
GFXBuffer display2 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 37, 34, -1));
GFXBuffer display3 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 44, 34, -1));
GFXBuffer display4 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 45, 34, -1));
GFXBuffer display5 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 46, 34, -1));
GFXBuffer display6 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 47, 34, -1));
GFXBuffer display7 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 48, 34, -1));
GFXBuffer display8 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 49, 34, -1));
GFXBuffer display9 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 50, 34, -1));
GFXBuffer display10 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 51, 34, -1));
GFXBuffer display11 = GFXBuffer(ST7735_TFTWIDTH_80, ST7735_TFTHEIGHT_160, display_t(&SPI, 52, 34, -1));
GFXBuffer *displays[] = {
  &display0,
  &display1,
  &display2,
  &display3,
  &display4,
  &display5,
  &display6,
  &display7,
  &display8,
  &display9,
  &display10,
  &display11,
};

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(115200);

  for(uint8_t i = 0; i < displayNumber; i++){
//    displays[i]->initS(60000000); //Unofficial modification for ST7735S displays
    displays[i]->initR(INITR_MINI160x80);
    displays[i]->setSPISpeed(60000000);
    displays[i]->setRotation(3);
    displays[i]->setTextWrap(0);
  }
  
  fps = 0;
  looped = 0;
  internal = 0;

  Serial.println(display0.initDMA(GFXBuffer::DMA0) ? "DMA0: On" : "DMA: Off");
  Serial.println(display1.initDMA(GFXBuffer::DMA1) ? "DMA1: On" : "DMA: Off");
  Serial.println(display2.initDMA(GFXBuffer::DMA2) ? "DMA2: On" : "DMA: Off");
  Serial.println(display3.initDMA(GFXBuffer::DMA3) ? "DMA3: On" : "DMA: Off");
  Serial.println(display4.initDMA(GFXBuffer::DMA4) ? "DMA4: On" : "DMA: Off");
  Serial.println(display5.initDMA(GFXBuffer::DMA5) ? "DMA5: On" : "DMA: Off");
  Serial.println(display6.initDMA(GFXBuffer::DMA6) ? "DMA6: On" : "DMA: Off");
  Serial.println(display7.initDMA(GFXBuffer::DMA7) ? "DMA7: On" : "DMA: Off");
  Serial.println(display8.initDMA(GFXBuffer::DMA8) ? "DMA8: On" : "DMA: Off");
  Serial.println(display9.initDMA(GFXBuffer::DMA9) ? "DMA9: On" : "DMA: Off");
  Serial.println(display10.initDMA(GFXBuffer::DMA10) ? "DMA10: On" : "DMA: Off");
  Serial.println(display11.initDMA(GFXBuffer::DMA11) ? "DMA11: On" : "DMA: Off");
}

void loop() {
  // put your main code here, to run repeatedly:
  while(true){
    screenUpdate();
  
    if(internalRefresh >= 500){ //Update or read peripherals here, lower refresh rate if needed
      internalRefresh -= 500;
      internal++;

      //Screen color is being changed here
      static uint8_t toggle = false;
      toggle++;
      if(toggle == 3){
        solidColor = display0.color565(0,0,0xFF);
        toggle = 0;
      }
      else if(toggle == 2){
        solidColor = display0.color565(0,0xFF,0);
      }
      else if(toggle == 1){
        solidColor = display0.color565(0xFF,0,0);
      }
    }
    
    looped++;
    if(oneSecond >= 1000){
      oneSecond -= 1000;
      Serial.print("FPS: ");
      Serial.print(fps);
      Serial.print("  Actual_FPS: ");
      Serial.print(fps_actual / 1.0 / displayNumber, 2);
      Serial.print("  Looped: ");
      Serial.print(looped);
      Serial.print("  Internal: ");
      Serial.println(internal);
      fps = 0;
      fps_actual = 0;
      looped = 0;
      internal = 0;
    }
  }
}

void screenUpdate(){  //Displays are refreshed here
  if(Serial.available()){
    targetFPS = (uint8_t)Serial.parseInt();
    Serial.print("Setting FPS to ");
    Serial.println(targetFPS);
  }
  if(screenRefresh >= (uint32_t)round((1000000/targetFPS/displayNumber))){  //Display refresh interval based on the number of displays and desired FPS
//    screenRefresh -= (uint32_t)round((1000000/targetFPS/displayNumber));
    screenRefresh = 0;

    drawDisplay(displays[displayToUpdate]);
    
    displayToUpdate++;
    if(displayToUpdate == displayNumber){
      displayToUpdate = 0;
      fps++;
    }
  }
}

void drawDisplay(GFXBuffer *display){  //Displays are drawn here (only make changes to the canvas here before drawing to your display for increased efficiency)
  //Use varibles to keep track of what to draw to the canvas then make the updates here accordingly
  display->fillScreen(solidColor);  //Change canvas color
  display->drawCircle(display->width()/2, display->height()/2, 20, 0xFFFF);
  fps_actual += display->display();  //Draw to display once canvas is ready
}
