#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE // 消除unsafe
#endif // !_CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <time.h>
#include <string.h>

void saveData(const char* dataFilename, const char* data, size_t dataSize, const char* direction, int isHex) {
    // 打开数据文件进行追加写入
    FILE* file = fopen(dataFilename, "a");
    if (file == NULL) {
        printf("无法打开文件 %s 进行写入\n", dataFilename);
        return;
    }

    // 获取当前时间
    time_t now;
    time(&now);
    struct tm local;
    localtime_s(&local, &now);
    // 写入时间戳和方向
    fprintf(file, "[%04d-%02d-%02d %02d:%02d:%02d] %s\n",
        local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
        local.tm_hour, local.tm_min, local.tm_sec, direction);
    // 写入数据
    if (isHex) {
        for (size_t i = 0; i < dataSize; ++i) {
            fprintf(file, "%02X ", (unsigned char)data[i]);
        }
    }
    else {
        for (size_t i = 0; i < dataSize; ++i) {
            fprintf(file, "%c", data[i]);
        }
    }




    // 换行以分隔每次追加的数据
    fprintf(file, "\n");
    fclose(file);
    //printf("数据成功保存到 %s\n", dataFilename);
}
