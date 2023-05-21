#include <esp_task_wdt.h> //Para el temporizador de viligancia de tareas
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"

TaskHandle_t xHandle;
// Creamos el método de la configuración del ADC y la tarea
esp_err_t set_adc(void);
esp_err_t create_task(void);

// Estructura de configuracion del temporizador de vigilancia de tareas
esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 3000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Máscara de bits de los dos núcleos
    .trigger_panic = true,
};

void TaskADC1(void *pvParameters) // Esta es una tarea
{
    // ESP_ERROR_CHECK(esp_task_wdt_add(NULL)); //Suscribimos la tarea al TWDT

    while (1)
    {
        int16_t adc_value = adc1_get_raw(ADC1_CHANNEL_4);
        printf("AD 4 (data cruda): %d\n\r", adc_value);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    // ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    set_adc();
    create_task();
}

esp_err_t set_adc(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);                        // Configuramos el máximo bits de 12 Resolución del ADC
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // GPIO 32 corresponde al canal 4, con maxima atenuación
    return ESP_OK;
}

esp_err_t create_task(void)
{
    xTaskCreatePinnedToCore(TaskADC1,
                            "Task1",
                            1024 * 2,
                            NULL,
                            1,
                            NULL,
                            0);
    return ESP_OK;
}