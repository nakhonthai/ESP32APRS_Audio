/* MIT License
 *
 * Copyright (c) 2020 Tino Hernandez
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef _ADAFRUIT_GFX_BUFFER_H
#define _ADAFRUIT_GFX_BUFFER_H
#include <Adafruit_GFX.h>

#if defined(__IMXRT1062__)
#include <DMAChannel.h>
#include <SPI.h>
#define ADAFRUIT_GFX_DMA_ENABLE (1)
typedef void (*gfxDMA_t)(void);
#else
#define ADAFRUIT_GFX_DMA_ENABLE (0)
#endif

template<typename display_t>
class Adafruit_GFX_Buffer : public GFXcanvas16, virtual public display_t
{
  public:
    Adafruit_GFX_Buffer(uint16_t width, uint16_t height, display_t display) : display_t(display), GFXcanvas16(width, height){
#if ADAFRUIT_GFX_DMA_ENABLE
#if defined(__IMXRT1062__)
    _spi = display_t::hwspi._spi;
    
    if(_spi == &SPI){
        _spi_struct = &IMXRT_LPSPI4_S;
        _spi_hardware = (SPIClass::SPI_Hardware_t*)&_spi->spiclass_lpspi4_hardware;
        gfxDMAChannel = &gfxDMAChannel0;
        gfxDMAActive = &gfxDMAActive0;
    }
    else if(_spi == &SPI1){
        _spi_struct = &IMXRT_LPSPI3_S;
        _spi_hardware = (SPIClass::SPI_Hardware_t*)&_spi->spiclass_lpspi3_hardware;
        gfxDMAChannel = &gfxDMAChannel1;
        gfxDMAActive = &gfxDMAActive1;
    }
    else if(_spi == &SPI2){
        _spi_struct = &IMXRT_LPSPI1_S;
        _spi_hardware = (SPIClass::SPI_Hardware_t*)&_spi->spiclass_lpspi1_hardware;
        gfxDMAChannel = &gfxDMAChannel2;
        gfxDMAActive = &gfxDMAActive2;
    }
#endif //__IMXRT1062__
#endif //ADAFRUIT_GFX_DMA_ENABLE
    }
    ~Adafruit_GFX_Buffer() { }
    
    bool display();

/*The following are overrides to make sure the canvas functions are used and not the display functions*/
    using GFXcanvas16::drawPixel;
    using GFXcanvas16::fillScreen;
    using GFXcanvas16::byteSwap;
    using GFXcanvas16::drawFastVLine;
    using GFXcanvas16::drawFastHLine;
    using GFXcanvas16::getPixel;
    using GFXcanvas16::getBuffer;
    
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){this->GFXcanvas16::fillRect(x, y, w, h, color);}
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){this->GFXcanvas16::drawRect(x, y, w, h, color);}
    void writePixel(int16_t x, int16_t y, uint16_t color){this->GFXcanvas16::writePixel(x, y, color);}
    void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){this->GFXcanvas16::writeFillRect(x, y, w, h, color);}
    void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color){this->GFXcanvas16::writeFastVLine(x, y, h, color);}
    void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color){this->GFXcanvas16::writeFastHLine(x, y, w, color);}
    void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){this->GFXcanvas16::writeLine(x0, y0, x1, y1, color);}
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){this->GFXcanvas16::drawLine(x0, y0, x1, y1, color);}
    void setRotation(uint8_t r){this->display_t::setRotation(0); this->GFXcanvas16::setRotation(r);}
    void invertDisplay(bool i){this->display_t::invertDisplay(i);}
    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color){this->GFXcanvas16::drawCircle(x0, y0, r, color);}
    void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color){this->GFXcanvas16::drawCircleHelper(x0, y0, r, cornername, color);}
    void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color){this->GFXcanvas16::fillCircle(x0, y0, r, color);}
    void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color){this->GFXcanvas16::fillCircleHelper(x0, y0, r, cornername, delta, color);}
    void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){this->GFXcanvas16::drawTriangle(x0, y0, x1, y1, x2, y2, color);}
    void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){this->GFXcanvas16::fillTriangle(x0, y0, x1, y1, x2, y2, color);}
    void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color){this->GFXcanvas16::drawRoundRect(x0, y0, w, h, radius, color);}
    void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color){this->GFXcanvas16::fillRoundRect(x0, y0, w, h, radius, color);}
    void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color){this->GFXcanvas16::drawBitmap(x, y, bitmap, w, h, color);}
    void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg){this->GFXcanvas16::drawBitmap(x, y, bitmap, w, h, color, bg);}
    void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color){this->GFXcanvas16::drawBitmap(x, y, bitmap, w, h, color);}
    void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg){this->GFXcanvas16::drawBitmap(x, y, bitmap, w, h, color, bg);}
    void drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color){this->GFXcanvas16::drawXBitmap(x, y, bitmap, w, h, color);}
    void drawYBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color){this->GFXcanvas16::drawYBitmap(x, y, bitmap, w, h, color);}
    void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h){this->GFXcanvas16::drawGrayscaleBitmap(x, y, bitmap, w, h);}
    void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h){this->GFXcanvas16::drawGrayscaleBitmap(x, y, bitmap, w, h);}
    void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h){this->GFXcanvas16::drawGrayscaleBitmap(x, y, bitmap, mask, w, h);}
    void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint8_t *mask, int16_t w, int16_t h){this->GFXcanvas16::drawGrayscaleBitmap(x, y, bitmap, mask, w, h);}
    void drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h){this->GFXcanvas16::drawRGBBitmap(x, y, bitmap, w, h);}
    void drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h){this->GFXcanvas16::drawRGBBitmap(x, y, bitmap, w, h);}
    void drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], const uint8_t mask[], int16_t w, int16_t h){this->GFXcanvas16::drawRGBBitmap(x, y, bitmap, mask, w, h);}
    void drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h){this->GFXcanvas16::drawRGBBitmap(x, y, bitmap, mask, w, h);}
    void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size){this->GFXcanvas16::drawChar(x, y, c, color, bg, size);}
    void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y){this->GFXcanvas16::drawChar(x, y, c, color, bg, size_x, size_y);}
    void getTextBounds(const char *string, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h){this->GFXcanvas16::getTextBounds(string, x, y, x1, y1, w, h);}
    void getTextBounds(const __FlashStringHelper *s, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h){this->GFXcanvas16::getTextBounds(s, x, y, x1, y1, w, h);}
    void getTextBounds(const String &str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h){this->GFXcanvas16::getTextBounds(str, x, y, x1, y1, w, h);}
    void setTextSize(uint8_t s){this->GFXcanvas16::setTextSize(s);}
    void setTextSize(uint8_t sx, uint8_t sy){this->GFXcanvas16::setTextSize(sx, sy);}
    void setFont(const GFXfont *f = NULL){this->GFXcanvas16::setFont(f);}
    void setCursor(int16_t x, int16_t y){this->GFXcanvas16::setCursor(x, y);}
    void setTextColor(uint16_t c){this->GFXcanvas16::setTextColor(c);}
    void setTextColor(uint16_t c, uint16_t bg){this->GFXcanvas16::setTextColor(c, bg);}
    void setTextWrap(bool w){this->GFXcanvas16::setTextWrap(w);}
    void cp437(bool x = true){this->GFXcanvas16::cp437(x);}
    int16_t width(){return this->GFXcanvas16::width();}
    int16_t height(){return this->GFXcanvas16::height();}
    uint8_t getRotation(void) const{return this->GFXcanvas16::getRotation();}
    int16_t getCursorX(void) const{return this->GFXcanvas16::getCursorX();}
    int16_t getCursorY(void) const{return this->GFXcanvas16::getCursorY();}

    size_t write(uint8_t b){return this->GFXcanvas16::write(b);}
    size_t write(const char *str){return this->GFXcanvas16::write(str);}
    size_t write(const uint8_t *buffer, size_t size){return this->GFXcanvas16::write(buffer, size);}
    int availableForWrite(void){return this->GFXcanvas16::availableForWrite();}
    void flush(){this->GFXcanvas16::flush();}
    size_t write(const char *buffer, size_t size){return this->GFXcanvas16::write(buffer, size);}
    size_t print(const String &s){return this->GFXcanvas16::print(s);}
    size_t print(char c){return this->GFXcanvas16::print(c);}
    size_t print(const char s[]){return this->GFXcanvas16::print(s);}
    size_t print(const __FlashStringHelper *f){return this->GFXcanvas16::print(f);}
    
    size_t print(uint8_t b){return this->GFXcanvas16::print(b);}
    size_t print(int n){return this->GFXcanvas16::print(n);}
    size_t print(unsigned int n){return this->GFXcanvas16::print(n);}
    size_t print(long n){return this->GFXcanvas16::print(n);}
    size_t print(unsigned long n){return this->GFXcanvas16::print(n);}
    
    size_t print(unsigned char n, int base){return this->GFXcanvas16::print(n, base);}
    size_t print(int n, int base){return this->GFXcanvas16::print(n, base);}
    size_t print(unsigned int n, int base){return this->GFXcanvas16::print(n, base);}
    size_t print(long n, int base){return this->GFXcanvas16::print(n, base);}
    size_t print(unsigned long n, int base){return this->GFXcanvas16::print(n, base);}
    
    size_t print(double n, int digits = 2){return this->GFXcanvas16::print(n, digits);}
    size_t print(const Printable &obj){return this->GFXcanvas16::print(obj);}
    size_t println(void){return this->GFXcanvas16::println();}
    size_t println(const String &s){return this->GFXcanvas16::println(s);}
    size_t println(char c){return this->GFXcanvas16::println(c);}
    size_t println(const char s[]){return this->GFXcanvas16::println(s);}
    size_t println(const __FlashStringHelper *f){return this->GFXcanvas16::println(f);}
    
    size_t println(uint8_t b){return this->GFXcanvas16::println(b);}
    size_t println(int n){return this->GFXcanvas16::println(n);}
    size_t println(unsigned int n){return this->GFXcanvas16::println(n);}
    size_t println(long n){return this->GFXcanvas16::println(n);}
    size_t println(unsigned long n){return this->GFXcanvas16::println(n);}
    
    size_t println(unsigned char n, int base){return this->GFXcanvas16::println(n, base);}
    size_t println(int n, int base){return this->GFXcanvas16::println(n, base);}
    size_t println(unsigned int n, int base){return this->GFXcanvas16::println(n, base);}
    size_t println(long n, int base){return this->GFXcanvas16::println(n, base);}
    size_t println(unsigned long n, int base){return this->GFXcanvas16::println(n, base);}
    
    size_t println(double n, int digits = 2){return this->GFXcanvas16::println(n, digits);}
    size_t println(const Printable &obj){return this->GFXcanvas16::println(obj);}
    int getWriteError(){return this->GFXcanvas16::getWriteError();}
    void clearWriteError(){this->GFXcanvas16::clearWriteError();}
    int printf(const char *format, ...){
      va_list ap;
      va_start(ap, format);
#ifdef __STRICT_ANSI__
      return 0;  // TODO: make this work with -std=c++0x
#else
      return vdprintf((int)this, format, ap);
#endif
    }
    int printf(const __FlashStringHelper *format, ...){
      va_list ap;
      va_start(ap, format);
#ifdef __STRICT_ANSI__
      return 0;
#else
      return vdprintf((int)this, (const char *)format, ap);
#endif
    }
/*End function overrides*/
    
#if ADAFRUIT_GFX_DMA_ENABLE
  public:
    bool initDMA(gfxDMA_t DMAFunction);
#define gfxDMAn(a)  static void DMA##a(){                       \
                        if(DMAnObject[(a)]){                    \
                            DMAnObject[(a)]->isrDMACom();       \
                        }                                       \
                        else{                                   \
                            gfxDMAChannel0.clearInterrupt();    \
                            gfxDMAChannel1.clearInterrupt();    \
                            gfxDMAChannel2.clearInterrupt();    \
                        }                                       \
                    } /*Unelegant to clear all DMA objects, but it should never happen*/
    gfxDMAn(0);
    gfxDMAn(1);
    gfxDMAn(2);
    gfxDMAn(3);
    gfxDMAn(4);
    gfxDMAn(5);
    gfxDMAn(6);
    gfxDMAn(7);
    gfxDMAn(8);
    gfxDMAn(9);
    gfxDMAn(10);
    gfxDMAn(11);
    static gfxDMA_t DMAn[];
    void changeSPIClock(uint8_t clock, uint8_t divisor){
#if defined(__IMXRT1062__)
        if(!_spi_hardware) return;
        if(!divisor) divisor = 1;
        _spi_hardware->clock_gate_register &= ~_spi_hardware->clock_gate_mask;
        CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_LPSPI_PODF_MASK | CCM_CBCMR_LPSPI_CLK_SEL_MASK)) |
            CCM_CBCMR_LPSPI_PODF(divisor & 7) | CCM_CBCMR_LPSPI_CLK_SEL(clock & 3);
        _spi_hardware->clock_gate_register |= _spi_hardware->clock_gate_mask;
#endif
    }
    bool displayComplete(){return gfxDMAChannel->complete();}
  private:
    static Adafruit_GFX_Buffer<display_t>* DMAnObject[sizeof(DMAn)/4];
    void isrDMACom();
    DMASetting gfxDMASetting;
    DMAChannel* gfxDMAChannel;
    static DMAChannel gfxDMAChannel0;
    static DMAChannel gfxDMAChannel1;
    static DMAChannel gfxDMAChannel2;
    gfxDMA_t gfxDMAFunction;
    bool* gfxDMAActive;
    static bool gfxDMAActive0;
    static bool gfxDMAActive1;
    static bool gfxDMAActive2;
    
#if defined(__IMXRT1062__)
    SPIClass* _spi = nullptr;
    IMXRT_LPSPI_t* _spi_struct = nullptr;
    uint32_t _spi_struct_TCR;
    SPIClass::SPI_Hardware_t* _spi_hardware = nullptr;
#endif
#endif
};

inline uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

template<typename display_t>
bool Adafruit_GFX_Buffer<display_t>::display(){  //Draw canvas to display
#if ADAFRUIT_GFX_DMA_ENABLE
    if(*gfxDMAActive && gfxDMAFunction && _spi){
        if(gfxDMAChannel->complete()){
            gfxDMAChannel->clearComplete();
            *gfxDMAChannel = gfxDMASetting;
            gfxDMAChannel->triggerAtHardwareEvent(_spi_hardware->tx_dma_channel);
            gfxDMAChannel->attachInterrupt(gfxDMAFunction);
            
            this->display_t::startWrite();
            uint8_t orientation = getRotation();
            switch(orientation){
              case 0:
              case 2:
                this->display_t::setAddrWindow(0, 0, this->GFXcanvas16::width(), this->GFXcanvas16::height());
                break;
              case 1:
              case 3:
                this->display_t::setAddrWindow(0, 0, this->GFXcanvas16::height(), this->GFXcanvas16::width());
                break;
            }
            _spi_struct_TCR = _spi_struct->TCR;
            _spi_struct->TCR = (_spi_struct->TCR & 0xfffff000) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK | LPSPI_TCR_BYSW;
            _spi_struct->DER = LPSPI_DER_TDDE;
            _spi_struct->SR = 0x3f00;
            byteSwap();
            if ((uint32_t)getBuffer() >= 0x20200000u)  arm_dcache_flush((void*)getBuffer(), width() * height() * 2);
            gfxDMAChannel->enable();
            return true;
        }
        return false;
    }
    else if(*gfxDMAActive){
        if(!gfxDMAChannel->complete()) return false;
    }
#endif
  uint8_t orientation = getRotation();
  switch(orientation){
    case 0:
    case 2:
      this->display_t::drawRGBBitmap(0,0,this->GFXcanvas16::getBuffer(), this->GFXcanvas16::width(), this->GFXcanvas16::height());
      break;
    case 1:
    case 3:
      this->display_t::drawRGBBitmap(0,0,this->GFXcanvas16::getBuffer(), this->GFXcanvas16::height(), this->GFXcanvas16::width());
      break;
  }
    return true;
}

#if ADAFRUIT_GFX_DMA_ENABLE
template<typename display_t>
DMAChannel Adafruit_GFX_Buffer<display_t>::gfxDMAChannel0;

template<typename display_t>
DMAChannel Adafruit_GFX_Buffer<display_t>::gfxDMAChannel1;

template<typename display_t>
DMAChannel Adafruit_GFX_Buffer<display_t>::gfxDMAChannel2;

template<typename display_t>
Adafruit_GFX_Buffer<display_t>* Adafruit_GFX_Buffer<display_t>::DMAnObject[sizeof(DMAn)/4];

template<typename display_t>
gfxDMA_t Adafruit_GFX_Buffer<display_t>::DMAn[] = {&DMA0, &DMA1, &DMA2, &DMA3, &DMA4, &DMA5, &DMA6, &DMA7, &DMA8, &DMA9, &DMA10, &DMA11};

template<typename display_t>
bool Adafruit_GFX_Buffer<display_t>::gfxDMAActive0 = false;

template<typename display_t>
bool Adafruit_GFX_Buffer<display_t>::gfxDMAActive1 = false;

template<typename display_t>
bool Adafruit_GFX_Buffer<display_t>::gfxDMAActive2 = false;

template<typename display_t>
bool Adafruit_GFX_Buffer<display_t>::initDMA(gfxDMA_t DMAFunction){
    if(!DMAFunction) return false;
    for(uint8_t i = 0; i < sizeof(DMAn) / 4; i++){
        if(DMAFunction == DMAn[i]) {
            DMAnObject[i] = this;
            gfxDMAFunction = DMAFunction;
            break;
        }
        else if(i == (sizeof(DMAn) / 4) - 1) return false;
    }
#if defined(__IMXRT1062__)
    if(!_spi) return false;
    
    gfxDMASetting.sourceBuffer((uint32_t*)getBuffer(), width() * height() / 2);
    gfxDMASetting.destination(_spi_struct->TDR);
    gfxDMASetting.transferSize(4);
    gfxDMASetting.transferCount(width() * height() / 2);
    gfxDMASetting.interruptAtCompletion();
    gfxDMASetting.disableOnCompletion();
    if(!*gfxDMAActive){

        *gfxDMAChannel = gfxDMASetting;
        
        gfxDMAChannel->triggerAtHardwareEvent(_spi_hardware->tx_dma_channel);
        gfxDMAChannel->attachInterrupt(gfxDMAFunction);
        
        this->display_t::startWrite();
        uint8_t orientation = getRotation();
        switch(orientation){
          case 0:
          case 2:
            this->display_t::setAddrWindow(0, 0, this->GFXcanvas16::width(), this->GFXcanvas16::height());
            break;
          case 1:
          case 3:
            this->display_t::setAddrWindow(0, 0, this->GFXcanvas16::height(), this->GFXcanvas16::width());
            break;
        }
        //  _spi_fcr_save = _pimxrt_spi->FCR;    // remember the FCR
        //    _pimxrt_spi->FCR = 0;    // clear water marks...
        _spi_struct_TCR = _spi_struct->TCR;
        _spi_struct->TCR = (_spi_struct->TCR & 0xfffff000) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK | LPSPI_TCR_BYSW;
        _spi_struct->DER = LPSPI_DER_TDDE;
        _spi_struct->SR = 0x3f00;
        if ((uint32_t)getBuffer() >= 0x20200000u)  arm_dcache_flush((void*)getBuffer(), width() * height() * 2);
        gfxDMAChannel->enable();
        
        *gfxDMAActive = true;
        return true;
    }
    else return true;
#elif
    return false;
#endif
}

template<typename display_t>
void Adafruit_GFX_Buffer<display_t>::isrDMACom(){
    gfxDMAChannel->clearInterrupt();
    _spi_struct->TCR = _spi_struct_TCR;
    while (_spi_struct->FSR & 0x1f);  // wait until this one is complete
    while (_spi_struct->SR & LPSPI_SR_MBF);  // wait until this one is complete
    this->display_t::endWrite();
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}
#endif
#endif //_ADAFRUIT_GFX_BUFFER_H
