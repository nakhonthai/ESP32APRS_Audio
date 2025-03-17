# Adafruit_GFX_Buffer
This library combines an Adafruit_GFX based display object and a GFXcanvas16 object into one.
This lets the display have a framebuffer thus allowing for more efficient use of SPI calls when drawing lots of things.

Syntax is as follows

```c++
Adafruit_GFX_Buffer<ADAFRUIT_DISPLAY_CLASS>(DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_CONSTRUCTOR);
```

Not much has to be changed to make use of the framebuffer, here's an example for an ST7735 based display. 
Here is the normal initializaition:

```c++
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

Adafruit_ST7735 display = Adafruit_ST7735(&SPI, 35, 34, 33);
```

Here is the framebuffer initialization:

```c++
#include <Adafruit_GFX_Buffer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

Adafruit_GFX_Buffer<Adafruit_ST7735> display = Adafruit_GFX_Buffer<Adafruit_ST7735>(80, 160,  Adafruit_ST7735(&SPI, 35, 34, 33)  );
```

When you are ready to send the framebuffer to your display you simply call the display function like this:

```c++
display.display();
```

See examples for more.
