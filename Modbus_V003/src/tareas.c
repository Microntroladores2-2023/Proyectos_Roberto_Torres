#include "modbus.h"
#include "driver/gpio.h"

extern UINT16_VAL MBHoldingRegister[maxHoldingRegister];
extern UINT16_VAL MBInputRegister[maxInputRegister];
extern UINT16_VAL MBCoils;
extern UINT16_VAL MBDiscreteInputs;

void TareaEntradaDatos(void *Parametro)
{
    int16_t datoInAux1 = -100;
    int16_t datoInAux2 = -20;
    int16_t datoInAux3 = 100;
    int16_t datoInAux4 = 0;
    uint8_t cont1 = 0;

    float_VAL datoFloat1;
    // float_VAL datoFloat2;

    datoFloat1.Val = -10.5;
    // datoFloat2.Val = 10.5;

    while (1)
    {
        // Registros 16 bit signo DIRECCIONES 0, 1, 2, 3

        MBInputRegister[0].Val = datoInAux1++;
        MBInputRegister[1].Val = datoInAux2++;
        MBInputRegister[2].Val = datoInAux3++;
        MBInputRegister[3].Val = datoInAux4++;

        // Registros FLOAT DIRECCIONES 100, 101 (Little-endian)

        MBInputRegister[100].byte.HB = datoFloat1.byte.Byte0;
        MBInputRegister[100].byte.LB = datoFloat1.byte.Byte1;
        MBInputRegister[101].byte.HB = datoFloat1.byte.Byte2;
        MBInputRegister[101].byte.LB = datoFloat1.byte.Byte3;

        datoFloat1.Val = datoFloat1.Val + 1.1;

        if (cont1++ == 50)
        {
            datoInAux1 = -100;
            datoInAux2 = -20;
            datoInAux3 = 100;
            datoInAux4 = 0;

            datoFloat1.Val = -10.5;
            cont1 = 0;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TareaSetCoils(void *Parametro)
{
    
#define Coil_1 GPIO_NUM_2
#define Coil_2 GPIO_NUM_16
#define Coil_3 GPIO_NUM_17
#define Coil_4 GPIO_NUM_20

    gpio_set_direction(Coil_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(Coil_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(Coil_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(Coil_4, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(Coil_1, MBCoils.bits.b5);
        gpio_set_level(Coil_2, MBCoils.bits.b6);
        gpio_set_level(Coil_3, MBCoils.bits.b7);
        gpio_set_level(Coil_4, MBCoils.bits.b8);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
