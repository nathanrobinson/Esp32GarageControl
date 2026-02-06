#pragma once

#include <Arduino.h>
#include "pin_config.h"

#ifdef ENABLE_DISPLAY

#include "Arduino_GFX_Library.h"

#define IMG_W 400
#define IMG_H 400
#define TEXT_BORDER_WIDTH 5

#endif

class Display
{
public:
    Display();
    ~Display();

    enum SCAN_LOCATION
    {
        WIFI = 0,
        CAR1 = 1,
        CAR2 = 2,
        BLE = 3
    };
    enum SIGNAL_STRENGTH
    {
        OFF = 0,
        POOR = 1,
        WEAK = 2,
        OK = 3,
        GOOD = 4,
    };

    void init();
    void updateWifi(String wifi);
    void showImage(uint8_t index); // show specific image index
    void updateConnection(SCAN_LOCATION location, SIGNAL_STRENGTH strength);
    void updateDistance(SCAN_LOCATION location, float meters);

private:
#ifdef ENABLE_DISPLAY
    void calculateArea();
    void preLoadBitmaps();
    void textAt(int16_t x, int16_t y, const char *text);
    void drawCurrentImage();
    void drawSignalIndicator(Display::SIGNAL_STRENGTH strength, const int bw, const int bh, Arduino_Canvas &canvas);
    void drawRawBMP565(uint8_t index);
    bool loadRaw565FromLittleFS(const char *path, uint16_t **out_buf, uint16_t img_w, uint16_t img_h);

    Arduino_ESP32RGBPanel *rgbpanel;
    Arduino_RGB_Display *gfx;

    static const uint8_t kImageCount = 4;
    const char *image_paths[kImageCount];
    uint16_t *bitmaps[kImageCount];
    uint8_t image_index;
    bool loaded;
    uint16_t startX;
    uint16_t startY;
    uint16_t wifiWidth;
    SIGNAL_STRENGTH lastStrength[4];
#endif
};

extern Display display;
