#include <stdint.h>
#include <main.h>
#include <Arduino.h>
#include "pin_config.h"
#include "display.h"
#include "settings.h"

#ifdef ENABLE_DISPLAY

#include <Wire.h>
#include "Arduino_GFX.h"
#include "Arduino_GFX_Library.h"
#include "load_file.h"

#else

#include "Adafruit_XCA9554.h" // Include the library header

Adafruit_XCA9554 io_expander;

#endif

// Global instance
Display display;

#ifdef ENABLE_DISPLAY

// Static factories returning single-instance placeholders for panel and display
static Arduino_ESP32RGBPanel *ST7262_Panel()
{
    static Arduino_ESP32RGBPanel inst(
        LCD_RGB_IO_DE /* DE */, LCD_RGB_IO_VSYNC /* VSYNC */, LCD_RGB_IO_HSYNC /* HSYNC */, LCD_RGB_IO_PCLK /* PCLK */,
        LCD_DATA_R0 /* R0 */, LCD_DATA_R1 /* R1 */, LCD_DATA_R2 /* R2 */, LCD_DATA_R3 /* R3 */, LCD_DATA_R4 /* R4 */,
        LCD_DATA_G0 /* G0 */, LCD_DATA_G1 /* G1 */, LCD_DATA_G2 /* G2 */, LCD_DATA_G3 /* G3 */, LCD_DATA_G4 /* G4 */, LCD_DATA_G5 /* G5 */,
        LCD_DATA_B0 /* B0 */, LCD_DATA_B1 /* B1 */, LCD_DATA_B2 /* B2 */, LCD_DATA_B3 /* B3 */, LCD_DATA_B4 /* B4 */,
        0 /* hsync_polarity */, LCD_RGB_TIMING_HFP /* hsync_front_porch */, LCD_RGB_TIMING_HPW /* hsync_pulse_width */, LCD_RGB_TIMING_HBP /* hsync_back_porch */,
        0 /* vsync_polarity */, LCD_RGB_TIMING_VFP /* vsync_front_porch */, LCD_RGB_TIMING_VPW /* vsync_pulse_width */, LCD_RGB_TIMING_VBP /* vsync_back_porch */,
        1 /* pclk_active_neg - IMPORTANT */, LCD_RGB_TIMING_FREQ_HZ /* int32_t prefer_speed */, false /* bool useBigEndian */,
        0 /* uint16_t de_idle_high */, 0 /* uint16_t pclk_idle_high */, LCD_RGB_BOUNCE_BUFFER_SIZE /* size_t bounce_buffer_size_px */
    );
    return &inst;
}

static Arduino_RGB_Display *ST7262_Display(Arduino_ESP32RGBPanel *panel)
{
    static Arduino_RGB_Display inst(
        LCD_WIDTH /* width */,
        LCD_HEIGHT /* height */,
        panel,
        0 /* rotation */,
        true /* auto_flush */
    );
    return &inst;
}

#endif

static void initializeLcdPins()
{

#ifdef ENABLE_DISPLAY

    // 0x0E Binary 00001110 (Pins 1, 2, 3 HIGH)
    // 0x0A Binary 00001010 (Pin 2 LOW, others HIGH)
    uint8_t pins = 0x0E;

    // 1. Enable Output Mode
    Wire.beginTransmission(ESP_PANEL_BOARD_EXPANDER_I2C_ADDRESS);
    Wire.write(0x01); // Bit 0 = 1 enables output pins
    if (Wire.endTransmission() != 0)
    {
        Serial.println("CH422G not found at 0x20!");
        return;
    }

    // 3. Drive Pins HIGH: LCD_RST (3), LCD_BL (2), TP_RST (1)
    // Register 0x38 controls the output levels of EXIO0-EXIO7
    // Binary 00001110 (0x0E) sets pins 1, 2, and 3 to HIGH
    Wire.beginTransmission(ESP_PANEL_BOARD_EXPANDER_CONTROL_ADDRESS);
    Wire.write(pins);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("CH422G not found at 0x38!");
        return;
    }

#else
    // Initialize the I/O expander
    if (!io_expander.begin(ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_110))
    {
        Serial.printf("TCA9554 expander not found at %2x\n", ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_110);
    }
    else
    {
        Serial.println("TCA9554 expander found successfully!");

        // Set the backlight pin as an OUTPUT
        io_expander.pinMode(IO_EXPANDER_PIN_NUM_1, OUTPUT);

        // Turn the backlight ON initially
        io_expander.digitalWrite(IO_EXPANDER_PIN_NUM_1, LOW);
    }
#endif
}

// Constructor sets up paths and initial state; heavy init happens in init()
Display::Display()
#ifdef ENABLE_DISPLAY
    : rgbpanel(nullptr), gfx(nullptr), image_index(0), loaded(false)
#endif
{
#ifdef ENABLE_DISPLAY
    image_paths[0] = "/GarageClosed.raw";
    image_paths[1] = "/GarageOpening.raw";
    image_paths[2] = "/GarageOpen.raw";
    image_paths[3] = "/GarageClosing.raw";
    for (int i = 0; i < kImageCount; ++i)
        bitmaps[i] = nullptr;
    for (int i = 0; i <= 4; ++i)
        lastStrength[i] = OFF;
#endif
}

Display::~Display()
{
#ifdef ENABLE_DISPLAY
    // free loaded bitmaps
    for (int i = 0; i < kImageCount; ++i)
    {
        if (bitmaps[i])
            delete[] bitmaps[i];
    }
    // `gfx` and `rgbpanel` come from static factories now; do not delete them here.
    gfx = nullptr;
    rgbpanel = nullptr;
#endif
}

void Display::init()
{
    delay(120); // Give the LCD controller time to wake up

    initializeLcdPins();

    delay(120); // Give the LCD controller time to wake up

#ifdef ENABLE_DISPLAY
    calculateArea();

    // create bus and gfx
    // 1. Define the RGB Panel Bus with your hardware pinout (use static factory)
    rgbpanel = ST7262_Panel();

    // 2. Initialize the Display class (use static factory)
    gfx = ST7262_Display(rgbpanel);

    if (!gfx->begin(LCD_RGB_TIMING_FREQ_HZ))
    {
        Serial.println("GFX initialization failed! Check PSRAM settings (must be OPI).");
    }
    else
    {
        Serial.println("GFX Success! Writing pixels...");
    }

    // gfx->setBrightness(255);
    gfx->fillScreen(RGB565_BLACK);
    gfx->setTextColor(RGB565_WHITESMOKE);
    textAt(250, 240, "Loading board");
    preLoadBitmaps();

    if (loaded)
    {
        // gfx->setBrightness(settings.DisplayBrightness());
        gfx->fillScreen(RGB565_BLACK);
        drawCurrentImage();
    }

    wifiWidth = 300;
    textAt(40, 50, "WiFi:");
    updateConnection(WIFI, OFF);
    textAt(40, 120, "Car 1:");
    updateConnection(CAR1, OFF);
    textAt(40, 190, "Car 2:");
    updateConnection(CAR2, OFF);
    textAt(40, 260, "BLE:");
    updateConnection(BLE, OFF);
#endif
}

void Display::updateWifi(String wifi)
{
#ifdef ENABLE_DISPLAY
    uint16_t wifiLength = wifi.length();
    uint16_t newWifiWidth = 300 + (wifiLength <= 9 ? 0 : ((wifiLength - 9) * 17) + 30);
    if (newWifiWidth > 740)
        newWifiWidth = 740;

    const int16_t x = 40;
    const int16_t y = 50;
    const int16_t w = wifiWidth > newWifiWidth ? wifiWidth : newWifiWidth; // safe width to clear previous text
    const int16_t h = 44;                                                  // safe height for textSize(4)

    wifiWidth = newWifiWidth;

    // Clear the previous text area and redraw new text
    // Render into an offscreen 16-bit canvas, then push as one bitmap
    Arduino_Canvas canvas(w, h, gfx);
    canvas.begin(GFX_SKIP_OUTPUT_BEGIN);
    canvas.fillRect(0, 0, w, h, RGB565_BLACK);
    canvas.setTextColor(RGB565_WHITESMOKE);
    canvas.setTextSize(4);
    canvas.setCursor(0, 0);
    canvas.print(wifi.c_str());

    // Blit the composed canvas to the display
    gfx->draw16bitRGBBitmap(x, y, canvas.getFramebuffer(), w, h);
#endif
}

void Display::showImage(uint8_t index)
{
#ifdef ENABLE_DISPLAY
    if (!loaded)
        return;
    if (index >= kImageCount)
        return;
    image_index = index;
    drawCurrentImage();

    // After drawing the garage image, redraw connection indicators
    updateConnection(SCAN_LOCATION::WIFI, lastStrength[0]);
#endif
}

void Display::updateConnection(SCAN_LOCATION location, SIGNAL_STRENGTH strength)
{
#ifdef ENABLE_DISPLAY
    // remember latest strength for redraws when images change
    lastStrength[static_cast<int>(location)] = strength;

    int16_t y = 20 + (location * 70);
    // clear indicator area
    const int bx = location == SCAN_LOCATION::WIFI ? wifiWidth : 300;
    const int bw = 60;
    const int bh = 60;
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
#endif
}

void Display::updateDistance(SCAN_LOCATION location, float meters)
{
#ifdef ENABLE_DISPLAY
    int16_t y = 50 + (location * 70);
    // clear indicator area
    const int bx = 200;
    const int bw = 75;
    const int bh = 40;

    uint16_t color;
    if (meters >= 5)
        color = RGB565_SPRINGGREEN;
    else if (meters >= 10)
        color = RGB565_YELLOWGREEN;
    else if (meters >= 15)
        color = RGB565_ORANGE;
    else if (meters >= 20)
        color = RGB565_RED;

    // Render the indicator into an offscreen 16-bit canvas, then push it as a single bitmap
    Arduino_Canvas canvas(bw, bh, gfx);

    // ensure canvas framebuffer allocated
    canvas.begin(GFX_SKIP_OUTPUT_BEGIN);
    // clear canvas
    canvas.fillRect(0, 0, bw, bh, RGB565_BLACK);
    canvas.setTextColor(color);
    canvas.setTextSize(3);
    canvas.setCursor(0, 0);
    canvas.printf("%.1fm", meters);

    // Push the composed canvas to the display in one bitmap call
    gfx->draw16bitRGBBitmap(bx, y, canvas.getFramebuffer(), bw, bh);
#endif
}

////////////////////////////////////////////////
///           Private Methods                ///
////////////////////////////////////////////////

#ifdef ENABLE_DISPLAY

void Display::calculateArea()
{
    startX = (LCD_WIDTH - IMG_W) - 10;
    startY = (LCD_HEIGHT - IMG_H) - 20;
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

void Display::textAt(int16_t x, int16_t y, const char *text)
{
    gfx->setCursor(x, y);
    gfx->setTextSize(4);
    gfx->println(text);
}

void Display::drawCurrentImage()
{
    drawRawBMP565(image_index);
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

#endif