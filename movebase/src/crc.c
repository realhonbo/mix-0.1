#include "stm32f1xx_hal.h"


uint32_t calculate_crc32(const uint8_t *data, int length)
{
    uint32_t crc = 0xFFFFFFFF;

    while (length--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}