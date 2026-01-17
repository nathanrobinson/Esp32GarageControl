#include "sound.h"
#include <Wire.h>
#include "ESP_I2S.h"
#include "es8311.h"
#include <math.h>
#include "settings.h"
#include "load_file.h"
I2SClass i2s;

Sound sound; // global instance

Sound::~Sound()
{
    if (click_pcm_8k)
    {
        delete[] click_pcm_8k;
        click_pcm_8k = nullptr;
        click_pcm_8k_length = 0;
    }

    if (chime_pcm_8k)
    {
        delete[] chime_pcm_8k;
        chime_pcm_8k = nullptr;
        chime_pcm_8k_length = 0;
    }
}

Sound::Sound()
    : _initialized(false) {}

bool Sound::init()
{
    // Enable power amplifier if board exposes PA pin
    pinMode(PA, OUTPUT);
    digitalWrite(PA, HIGH);

    i2s.setPins(I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO, I2S_DI_IO, I2S_MCK_IO);
    if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_BOTH))
    {
        Serial.println("Failed to initialize I2S bus!");
        return false;
    }

    es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);

    if (!es_handle)
    {
        Serial.println("es8311 create failed");
        return false;
    }

    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = SAMPLE_RATE * 256,
        .sample_frequency = SAMPLE_RATE};

    es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency);
    es8311_voice_volume_set(es_handle, settings.Volume(), NULL);

    loadSoundFiles();

    _initialized = true;

    cowbell();

    return _initialized;
}

void Sound::loadSoundFiles()
{
    click_pcm_8k_length = loadFile.loadFile("/click.raw", &click_pcm_8k);
    chime_pcm_8k_length = loadFile.loadFile("/cowbell.raw", &chime_pcm_8k);
}

void Sound::click()
{
    if (_initialized && click_pcm_8k && click_pcm_8k_length)
    {
        i2s.write(reinterpret_cast<uint8_t *>(click_pcm_8k), click_pcm_8k_length * sizeof(int16_t));
    }
}

void Sound::cowbell()
{
    if (_initialized && chime_pcm_8k && chime_pcm_8k_length)
    {
        i2s.write(reinterpret_cast<uint8_t *>(chime_pcm_8k), chime_pcm_8k_length * sizeof(int16_t));
    }
}