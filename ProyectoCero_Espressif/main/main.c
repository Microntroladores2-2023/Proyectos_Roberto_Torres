#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h" //Es una biblioteca de registro

/*Existen los siguientes tipos de verbosidad:
 * Error mas bajo se usa ESP_LOGE()
 * Advertencia de usa ESP_LOW()
 * Información se usa ESP_LOGI()
 * Depurar se usa ESP_LOGD()
 * Detallado mas alto se usa ESP_LOGV()
 */

// Se crean los prototipos de las funciones
esp_err_t create_task(void);
void Tarea1(void *pvParameters);
void Tarea2(void *pvParameters);
void Tarea3(void *pvParameters);
void Tarea4(void *pvParameters);

static const char *TAG = "Estado"; // Se crea una variable estática de tipo char en donde se define la variable TAG

void Tarea1(void *pvParameters) // Esta es una tarea
{

    while (1) // Una tarea nunca regresará ni saldrá
    {
        printf("Hola Mundo Tarea1 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID()); // xPortGetCoreID() es una funcion que indica en que nucleo se ejecuta
        // Aqui se puede utilizar una de las macros de la libreria de registro, usaremos la de informacion ESP_LOGI()
        ESP_LOGI(TAG, "Hola Mundo Tarea1 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Tarea3(void *pvParameters) // Esta es una tarea
{

    while (1) // Una tarea nunca regresará ni saldrá
    {
        printf("Hola Mundo Tarea3 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
        ESP_LOGI(TAG, "Hola Mundo Tarea3 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Tarea2(void *pvParameters) // Esta es una tarea
{

    while (1) // Una tarea nunca regresará ni saldrá
    {
        printf("Hola Mundo Tarea2 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
        ESP_LOGI(TAG, "Hola Mundo Tarea2 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Tarea4(void *pvParameters) // Esta es una tarea
{

    while (1) // Una tarea nunca regresará ni saldrá
    {
        printf("Hola Mundo Tarea4 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
        ESP_LOGI(TAG, "Hola Mundo Tarea4 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
create_task();
}

esp_err_t create_task(void)
{
    xTaskCreatePinnedToCore(Tarea1,
                            "Task1",
                            1024 * 2,
                            NULL,
                            1,
                            NULL,
                            0);

    xTaskCreatePinnedToCore(Tarea2,
                            "Task2",
                            1024 * 2,
                            NULL,
                            2,
                            NULL,
                            1);

    xTaskCreatePinnedToCore(Tarea3,
                            "Task3",
                            1024 * 2,
                            NULL,
                            3,
                            NULL,
                            0);

    xTaskCreatePinnedToCore(Tarea4,
                            "Task4",
                            1024 * 2,
                            NULL,
                            4,
                            NULL,
                            1);
    
    return ESP_OK;
}