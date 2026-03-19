
#include "load_file.h"
#include <LittleFS.h>
#include <Arduino.h>
#include <new>

// Define the global instance referenced by other translation units.
LoadFile loadFile;

LoadFile::LoadFile()
{
    if (!LittleFS.begin())
    {
        Serial.println("LittleFS mount failed");
        while (1)
            ;
    }
}

size_t LoadFile::loadIndianFile(const char *path, uint16_t **out_buf)
{
    if (out_buf)
        *out_buf = nullptr;

    if (path == nullptr)
        return 0;

    if (!LittleFS.exists(path))
        return 0;

    File f = LittleFS.open(path, "r");
    if (!f)
        return 0;

    size_t size = f.size();
    if (size == 0 || (size % 2) != 0)
    { // file must contain whole 16-bit values
        f.close();
        return 0;
    }

    size_t count = size / 2;
    uint16_t *data = new (std::nothrow) uint16_t[count];
    if (!data)
    {
        f.close();
        return 0;
    }

    // Read entire file in one bulk call for performance and to avoid long
    // per-byte overhead which can trigger the task watchdog when running in setup().
    size_t bytesToRead = count * 2;
    size_t bytesRead = f.read(reinterpret_cast<uint8_t *>(data), bytesToRead);
    if (bytesRead != bytesToRead)
    {
        delete[] data;
        f.close();
        return 0;
    }

    f.close();
    if (out_buf)
        *out_buf = data;
    return count;
}

size_t LoadFile::loadFile(const char *path, int16_t **out_buf)
{
    if (out_buf)
        *out_buf = nullptr;

    if (path == nullptr)
        return 0;

    if (!LittleFS.exists(path))
        return 0;

    File f = LittleFS.open(path, "r");
    if (!f)
        return 0;

    size_t size = f.size();
    if (size == 0 || (size % 2) != 0)
    { // file must contain whole 16-bit values
        f.close();
        return 0;
    }

    size_t count = size / 2;
    int16_t *data = new (std::nothrow) int16_t[count];
    if (!data)
    {
        f.close();
        return 0;
    }

    f.read(reinterpret_cast<uint8_t *>(data), size);

    f.close();
    if (out_buf)
        *out_buf = data;
    return count;
}
