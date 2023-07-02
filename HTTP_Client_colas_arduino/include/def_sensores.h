#include "freertos/FreeRTOS.h"

typedef struct
{
    float acelx;
    float acely;
    float acelz;
    float gcelx;
    float gcely;
    float gcelz;
    uint8_t PIR;
    float ULTRA;
} Dato;