#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdio.h>
#include <time.h>
#include <string.h>

void saveData(const char* dataFilename, const char* data, size_t dataSize, const char* direction, int isHex);

#endif // DATA_LOGGER_H
