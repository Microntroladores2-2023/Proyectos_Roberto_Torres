#include "Arduino.h"

void Blink(void *pvParameters) // Esta es una tarea
{
#define LED_PIN (gpio_num_t)2

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    int ON = 0;

    while (1) // Una tarea nunca regresará ni saldrá
    {
        ON = !ON;
        gpio_set_level(LED_PIN, ON);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    xTaskCreatePinnedToCore(Blink,
                            "Parpadero",
                            1024 * 2,
                            NULL,
                            1,
                            NULL,
                            0);
}

void loop()
{
}