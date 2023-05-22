#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static char *mensaje_inicial = "Programa para realizar la comunicacion serial del ESP32 \n";
static char *mensaje_inicial_2 = "El Programa tiene como objetivo encender y apagar los leds R G y B \n";
static char *mensaje_inicial_3 = "Para encender o apagar los leds basta con ingresar sus iniciales \n";

static QueueHandle_t uart0_queue; // Con esta variable indicamos que crearemos una cola, es una cantidad de mensajes que vendrán

void protocolo1Serial(uint8_t *ByteArray, uint16_t Length);

#define tamBUFFER 1024

uint8_t estado_led_R = 0;
uint8_t estado_led_G = 0;
uint8_t estado_led_B = 0;

// Definición de pines para los leds

#define led_R 13
#define led_G 12
#define led_B 14

// Prototipo de la función para iniciar la configuración de los leds

esp_err_t init_led(void);

// Función del evento con UART

void TareaEventosUART0(void *Parametro)
{
    uart_event_t evento;

    uint8_t *datoRX = (uint8_t *)malloc(tamBUFFER); // Puntero a un arreglo donde se guardarán los datos, en forma dinámica
    uart_write_bytes(UART_NUM_0, mensaje_inicial, strlen(mensaje_inicial));
    uart_write_bytes(UART_NUM_0, mensaje_inicial_2, strlen(mensaje_inicial_2));
    uart_write_bytes(UART_NUM_0, mensaje_inicial_3, strlen(mensaje_inicial_3));
    while (1)
    {
        if (xQueueReceive(uart0_queue, (void *)&evento, (TickType_t)portMAX_DELAY)) //(xHandle, puntero del evento, tiempo de retraso)
        {                                                                           // (void *) se queda esperando por siempre una recepción. Asi solo entra a este bucle cuando recibe un mensaje
            bzero(datoRX, tamBUFFER);                                               // Función que llena de ceros sus parámetros
            if (evento.type == UART_DATA)                                           // Los datos llegan en la cola por el puerto serial
            {
                uart_read_bytes(UART_NUM_0, datoRX, evento.size, portMAX_DELAY); // Si llegan los datos lee los datos en el controlador uart0
              
                protocolo1Serial(datoRX, evento.size);
   
            }
        }
    }

    free(datoRX);
    datoRX = NULL;
    vTaskDelete(NULL); // Se limpia el buffer de entrada para evitar overflow
}



// Función Para iniciar el UART0

void initUART0()
{
    uart_config_t configUART0 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &configUART0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // Configuro los pines del UART
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, tamBUFFER * 2, tamBUFFER * 2, 20, &uart0_queue, 0));                       // config el UART por promedio de colas por queue UARTPINOHANGE deja el pin por defecto

    xTaskCreatePinnedToCore(TareaEventosUART0, "Tarea_para_UART0", 1024 * 5, NULL, 12, NULL, 1);
}

void app_main()
{
    init_led();
    initUART0();
}

// Función del protocolo serial

void protocolo1Serial(uint8_t *ByteArray, uint16_t Length)
{

    uint8_t estado = ByteArray[0]; // maquina de estado

    switch (estado)
    {
    case 'R':

    estado_led_R = !estado_led_R;
    gpio_set_level(led_R, estado_led_R);

        break;

    case 'G':

    estado_led_G = !estado_led_G;
    gpio_set_level(led_G, estado_led_G);

        break;

    case 'B':

    estado_led_B = !estado_led_B;
    gpio_set_level(led_B, estado_led_B);


        break;

    default:

        gpio_set_level(led_R, 0);
        gpio_set_level(led_G, 0);
        gpio_set_level(led_B, 0);

        break;
    }
}

// Iinicio de la función de la configuración de puertos

esp_err_t init_led(void)
{
    // Se resetea cualquier valor en los pines

    gpio_reset_pin(led_R);
    gpio_reset_pin(led_G);
    gpio_reset_pin(led_B);

    // Se selecciona el modo, en este caso como salida

    gpio_set_direction(led_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_B, GPIO_MODE_OUTPUT);

    // Se colocan en estado cero los pines al iniciar

    gpio_set_level(led_R, estado_led_R);
    gpio_set_level(led_G, estado_led_G);
    gpio_set_level(led_B, estado_led_B);

    return ESP_OK;
}