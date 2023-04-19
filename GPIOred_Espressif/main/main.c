#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN 2
int estado_led = 0;

// Funciones de prototipo
void Blink(void *pvParameters);
esp_err_t config_pin(void);
esp_err_t create_task(void);

void app_main(void)
{
    config_pin();
    create_task();
}

esp_err_t config_pin(void)
{
    gpio_reset_pin(LED_PIN); //Reseteo del pin
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); //Modo salido el pin 2
    gpio_set_level(LED_PIN, estado_led); // Se coloca en estado 0 el pin al iniciar

    return ESP_OK;
}

void Blink(void *pvParameters)
{
    while (1)
    {
        estado_led = !estado_led;
        gpio_set_level(LED_PIN, estado_led);
        vTaskDelay(pdMS_TO_TICKS(1000)); //Retardo de 1 segundo
    }

}

esp_err_t create_task(void)
{
    xTaskCreatePinnedToCore(Blink, //Nombre de la función que ejecuta la tarea
                            "Parpadeo", //Nombre arbitrario
                            1024 * 2, //Tamaño de la memoria que ocupa la tarea
                            NULL, //Puntero nulo
                            1, //Prioridad 1
                            NULL, //Puntero nulo
                            0); // Que se ejecute en el Procesador 0

    return ESP_OK;
}