#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t init_led(void);     // Función prototipo de la configuración de pines
esp_err_t create_tasks(void); // Función prototipo de las tareas

#define PUSH_BUTTON_PIN 14
#define LED_PIN 2

TaskHandle_t xHandle;

uint16_t tiempo_Led = 1000; // variable global

void Boton(void *pvParameters)
{

    uint8_t estado_maquina = 0;

    while (1)
    {
        // presiona boton y suelta
        while (gpio_get_level(PUSH_BUTTON_PIN) == 0)
            vTaskDelay(10 / portTICK_PERIOD_MS); // boton presionado

        while (gpio_get_level(PUSH_BUTTON_PIN) == 1)
            vTaskDelay(10 / portTICK_PERIOD_MS); // boton sin presionar

        switch (estado_maquina)
        {
        case 0:
            tiempo_Led = 100;
            estado_maquina = 1;
            break;

        case 1:
            tiempo_Led = 2000;
            estado_maquina = 2;
            break;

        case 2:
            vTaskSuspend(xHandle);
            estado_maquina = 3;
            break;

        case 3:
            vTaskResume(xHandle);
            tiempo_Led = 50;
            estado_maquina = 4;
            break;

        case 4:
            tiempo_Led = 500;
            estado_maquina = 0;
            break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Blink(void *pvParameters) // Esta es una tarea
{
    int ON = 0;

    while (true) // Una tarea nunca regresará ni saldrá
    {
        ON = !ON;
        gpio_set_level(LED_PIN, ON);
        vTaskDelay(tiempo_Led / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    init_led();
    create_tasks();
}

esp_err_t init_led(void) // Configuracion de los pines
{
    gpio_reset_pin(PUSH_BUTTON_PIN);
    gpio_set_direction(PUSH_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    return ESP_OK;
}

esp_err_t create_tasks(void)
{
    xTaskCreatePinnedToCore(Boton, //Nombre de la función que ejecuta la tarea
                            "Boton", //Nombre arbitrario
                            1024 * 2, //Tamaño de memoria que ocupa la tarea
                            NULL, //puntero estatico de 8 bits nulo
                            1,  //Prioridad 1
                            NULL, //Puntero handle nulo
                            0); // Se le asigna el core 0

    xTaskCreatePinnedToCore(Blink,
                            "Parpadeo",
                            1024 * 2,
                            NULL,
                            1,
                            &xHandle,
                            0);
                            
    return ESP_OK;
}