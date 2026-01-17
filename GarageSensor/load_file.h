#pragma once

#include <stdint.h>
#include <stddef.h>

class LoadFile
{
public:
    LoadFile();
    // Loads `path` from LittleFS and returns the number of uint16_t elements
    // read. On success `*out_buf` will be set to a newly allocated
    // uint16_t array (caller must free with `freeBuffer`). Returns 0 on
    // failure.
    size_t loadIndianFile(const char *path, uint16_t **out_buf);
    // Loads `path` from LittleFS and returns the number of uint16_t elements
    // read. On success `*out_buf` will be set to a newly allocated
    // int16_t array (caller must free with `freeBuffer`). Returns 0 on
    // failure.
    size_t loadFile(const char *path, int16_t **out_buf);
};

extern LoadFile loadFile;
