#include "tareas.h"

extern TaskHandle_t xHandle;

uint16_t tiempo_led = 1000; // variable global

void Boton(void *pvParameters)
{
#define PUSH_BUTTON_PIN (gpio_num_t)14
    enum ESTADOS
    {
        velocidad_lenta,
        velocidad_rapida,
        suspension_blink,
        resumen_BLINK,
        velocidad_media
    };

    uint8_t estado_maquina = velocidad_lenta;
    gpio_set_direction(PUSH_BUTTON_PIN, GPIO_MODE_INPUT); // Funcion que recibe como parametro numero del pin y modo

    while (1)
    {
        // se presiona boton y se suelta
        while (gpio_get_level(PUSH_BUTTON_PIN) == 0) // Si se presiona boton
            vTaskDelay(pdMS_TO_TICKS(10));           // se produce un delay de 10ms

        while (gpio_get_level(PUSH_BUTTON_PIN) == 1) // Si se dejó de presionar el botón
            vTaskDelay(pdMS_TO_TICKS(10));           // retraso de 10ms

        switch (estado_maquina)
        {
        case velocidad_lenta:
            tiempo_led = 2000;
            estado_maquina = 2;
            break;

        case velocidad_rapida:
            tiempo_led = 100;
            estado_maquina = 0;
            break;

        case suspension_blink:
            vTaskSuspend(xHandle); // Se suspende la tarea en ejecucion
            estado_maquina = 3;
            break;

        case resumen_BLINK:
            vTaskResume(xHandle);
            tiempo_led = 50;
            estado_maquina = 4;
            break;

        case velocidad_media:
            tiempo_led = 500;
            estado_maquina = 1;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Blink(void *pvParameters)
{
    #define LED_PIN (gpio_num_t)2
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    int ON = 0;

    while(true) //Una tarea nunca regresará ni saldrá
    {
        ON = !ON;
        gpio_set_level(LED_PIN, ON);
        vTaskDelay(pdMS_TO_TICKS(tiempo_led));
    }
}