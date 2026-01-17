#pragma once

#include <Arduino.h>
#include "pin_config.h"
#include "es8311.h"

#define SAMPLE_RATE 8000

#define I2C_NUM 0

#define I2S_MCK_IO 16
#define I2S_BCK_IO 9
#define I2S_DI_IO 10
#define I2S_WS_IO 45
#define I2S_DO_IO 8

class Sound
{
public:
    Sound();
    ~Sound();

    // Initialize the sound subsystem. If `wire` is provided it will be
    // used for ES8311 I2C control. `i2c_addr` defaults to 0x18. If
    // `fallbackPin` >= 0 a PWM fallback will be attached to that pin
    // for click/beep when the codec isn't available or when a simple
    // tone is preferred.
    bool init();

    void click();

    void cowbell();

private:
    void loadSoundFiles();
    bool _initialized;

    int16_t *chime_pcm_8k;
    size_t chime_pcm_8k_length;
    int16_t *click_pcm_8k;
    size_t click_pcm_8k_length;
};

extern Sound sound;
