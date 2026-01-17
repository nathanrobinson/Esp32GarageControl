#include <stdint.h>
#include "Arduino_GFX.h"
#include <Arduino.h>
#include "pin_config.h"
#include "Arduino_GFX_Library.h"
#include "display.h"
#include "settings.h"
#include "load_file.h"

// Global instance
Display display;

// Constructor sets up paths and initial state; heavy init happens in init()
Display::Display()
    : bus(nullptr), gfx(nullptr), image_index(0), loaded(false)
{
    image_paths[0] = "/GarageClosed.raw";
    image_paths[1] = "/GarageOpening.raw";
    image_paths[2] = "/GarageOpen.raw";
    image_paths[3] = "/GarageClosing.raw";
    for (int i = 0; i < kImageCount; ++i)
        bitmaps[i] = nullptr;
}

Display::~Display()
{
    // free loaded bitmaps
    for (int i = 0; i < kImageCount; ++i)
    {
        if (bitmaps[i])
            delete[] bitmaps[i];
    }
    // delete gfx and bus
    delete gfx;
    delete bus;
}

void Display::init()
{
    calculateArea();
    lastStateChange = millis() / 1000;

    // create bus and gfx
    bus = new Arduino_ESP32QSPI(
        LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
        LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

    gfx = new Arduino_SH8601(
        bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, LCD_WIDTH /* width */, LCD_HEIGHT /* height */);

    gfx->begin();
    gfx->setBrightness(255);
    gfx->fillScreen(RGB565_BLACK);
    gfx->setTextColor(RGB565_WHITESMOKE);
    textAt(30, 150, "Loading board");
    preLoadBitmaps();

    if (loaded)
    {
        gfx->setBrightness(settings.DisplayBrightness());
        gfx->fillScreen(RGB565_BLACK);
        drawCurrentImage();
    }

    textAt(20, 20, "Garage:");
    updateConnection(GARAGE, OFF);
    textAt(20, 70, "Driveway:");
    updateConnection(DRIVEWAY, OFF);
}

void Display::loop()
{
    long now = millis() / 1000;
    if ((now - lastStateChange) > DIM_BACKLIGHT_SECONDS)
    {
        enableDisplay(false);
    }
}

void Display::enableDisplay(bool on)
{
    if (on)
    {
        gfx->setBrightness(settings.DisplayBrightness());
        lastStateChange = millis() / 1000;
    }
    else
    {
        gfx->setBrightness(10);
    }
}

void Display::updateConnection(SCAN_LOCATION location, SIGNAL_STRENGTH strength)
{
    int16_t y = 20 + (location * 50);

    // clear indicator area
    const int bx = 290;
    const int bw = 40;
    const int bh = 40;

    // Render the indicator into an offscreen 16-bit canvas, then push it as a single bitmap
    Arduino_Canvas canvas(bw, bh, gfx);

    // ensure canvas framebuffer allocated
    canvas.begin(GFX_SKIP_OUTPUT_BEGIN);
    // clear canvas
    canvas.fillRect(0, 0, bw, bh, RGB565_BLACK);

    if (strength > OFF)
    {
        drawSignalIndicator(strength, bw, bh, canvas);
    }

    // Push the composed canvas to the display in one bitmap call
    gfx->draw16bitRGBBitmap(bx, y, canvas.getFramebuffer(), bw, bh);
}

void Display::drawSignalIndicator(Display::SIGNAL_STRENGTH strength, const int bw, const int bh, Arduino_Canvas &canvas)
{
    uint16_t color1, color2, color3, color4;
    switch (strength)
    {
    case GOOD:
        color1 = color2 = color3 = color4 = RGB565_SPRINGGREEN;
        break;
    case OK:
        color1 = color2 = color3 = RGB565_YELLOWGREEN;
        color4 = RGB565(76, 102, 24);
        break;
    case WEAK:
        color1 = color2 = RGB565_ORANGE;
        color3 = RGB565(166, 110, 0);
        color4 = RGB565(83, 55, 0);
        break;
    case POOR:
    default:
        color1 = RGB565_RED;
        color2 = RGB565(186, 0, 0);
        color3 = RGB565(124, 0, 0);
        color4 = RGB565(62, 0, 0);
        break;
    }
    // Draw Wi-Fi symbol into canvas using local coordinates
    const int cx = bw / 2; // center x relative to canvas
    const int cy = bh - 5; // baseline at bottom of canvas

    // filled dot
    canvas.fillCircle(cx, cy, 4, color1);

    // helper: draw an arc band given inner/outer radii
    auto drawArcBand = [&](int r1, int r2, uint16_t color)
    {
        canvas.drawArc(cx, cy, r1, r2, 200.0f, 340.0f, color);
        canvas.fillArc(cx, cy, r1, r2, 200.0f, 340.0f, color);
    };

    // draw three arcs above the dot (smaller to larger)
    drawArcBand(9, 14, color2);
    drawArcBand(19, 24, color3);
    drawArcBand(29, 34, color4);
}

void Display::textAt(int16_t x, int16_t y, const char *text)
{
    gfx->setCursor(x, y);
    gfx->setTextSize(4);
    gfx->println(text);
}

void Display::preLoadBitmaps()
{
    for (int i = 0; i < kImageCount; i++)
    {
        if (!loadRaw565FromLittleFS(image_paths[i], &bitmaps[i], IMG_W, IMG_H))
        {
            return;
        }
    }
    loaded = true;
    Serial.println("Bitmaps loaded");
}

void Display::drawCurrentImage()
{
    enableDisplay(true);
    drawRawBMP565(image_index);
}

void Display::showImage(uint8_t index)
{
    if (!loaded)
        return;
    if (index >= kImageCount)
        return;
    image_index = index;
    drawCurrentImage();
}

void Display::calculateArea()
{
    startX = (LCD_WIDTH - IMG_W) / 2;
    startY = (LCD_HEIGHT - IMG_H) - startX;
}

void Display::drawRawBMP565(const uint8_t index)
{
    if (!loaded)
        return;

    // Draw the buffered image using optimized bulk draw
    gfx->draw16bitRGBBitmap(startX, startY, bitmaps[index], IMG_W, IMG_H);
}

// Load a raw RGB565 image from LittleFS into a uint16_t buffer (2 bytes per pixel).
// Expects the file size to match expected_w * expected_h * 2. Allocates *out_buf
// (caller must delete[]). Returns true on success.
bool Display::loadRaw565FromLittleFS(const char *path, uint16_t **out_buf, uint16_t img_w, uint16_t img_h)
{
    size_t pixelCount = (size_t)img_w * (size_t)img_h;
    uint16_t *buffer = nullptr;
    size_t count = loadFile.loadIndianFile(path, &buffer);
    if (!buffer)
    {
        Serial.printf("Image file not found or failed to load: %s\n", path);
        return false;
    }

    if (count != pixelCount)
    {
        Serial.printf("Image size mismatch: %s\n", path);
        delete[] buffer;
        return false;
    }

    *out_buf = buffer;
    return true;
}