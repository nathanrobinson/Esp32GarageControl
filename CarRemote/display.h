#pragma once

#include <Arduino.h>
#include "pin_config.h"
#include "Arduino_GFX_Library.h"

#define IMG_W 300
#define IMG_H 300
#define TEXT_BORDER_WIDTH 5
#define DIM_BACKLIGHT_SECONDS 90

class Display
{
public:
    Display();
    ~Display();

    enum SCAN_LOCATION
    {
        GARAGE = 0,
        DRIVEWAY = 1
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
    void loop();
    void showImage(uint8_t index); // show specific image index
    void updateConnection(SCAN_LOCATION location, SIGNAL_STRENGTH strength);
    void drawSignalIndicator(SIGNAL_STRENGTH strength, const int bw, const int bh, Arduino_Canvas &canvas);
    void enableDisplay(bool on);

private:
    void calculateArea();
    void drawCurrentImage();
    void preLoadBitmaps();
    void drawRawBMP565(uint8_t index);
    bool loadRaw565FromLittleFS(const char *path, uint16_t **out_buf, uint16_t img_w, uint16_t img_h);
    void textAt(int16_t x, int16_t y, const char *text);

    Arduino_DataBus *bus;
    Arduino_SH8601 *gfx;

    static const uint8_t kImageCount = 4;
    const char *image_paths[kImageCount];
    uint16_t *bitmaps[kImageCount];
    uint8_t image_index;
    bool loaded;
    uint16_t startX;
    uint16_t startY;
    long lastStateChange;
};

extern Display display;
