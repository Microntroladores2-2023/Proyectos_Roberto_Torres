#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

static char dato[] = "Programa para realizar la comunicacion serial del ESP32";

static QueueHandle_t uart0_queue; //Con esta variable indicamos que crearemos una cola, es una cantidad de mensajes que vendrán

void protocolo1Serial(uint8_t *ByteArray, uint16_t Length);

#define tamBUFFER 1024

//**************************************
//*************** TASKs ****************
//**************************************

void TareaEventosUART0(void *Parametro)
{
    uart_event_t evento;
    uart_write_bytes(UART_NUM_0, (const char*) dato, 2048);
    uint8_t *datoRX = (uint8_t *)malloc(tamBUFFER); //Puntero a un arreglo donde se guardarán los datos, en forma dinámica

    while(1)
    {
        if (xQueueReceive(uart0_queue, (void *)&evento, (TickType_t)portMAX_DELAY)) //(xHandle, puntero del evento, tiempo de retraso)
        {                                                                           // (void *) se queda esperando por siempre una recepción. Asi solo entra a este bucle cuando recibe un mensaje 
            bzero(datoRX, tamBUFFER);  //Función que llena de ceros sus parámetros                                             
            if (evento.type == UART_DATA) //Los datos llegan en la cola por el puerto serial
            {
                uart_read_bytes(UART_NUM_0, datoRX, evento.size, portMAX_DELAY); //Si llegan los datos lee los datos en el controlador uart0
                // modbusSerial(datoRX, evento.size);

                //protocolo1Serial(datoRX, evento.size);

                uart_write_bytes(UART_NUM_0, (const char*) datoRX, evento.size);

                // vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }

    free(datoRX);
    datoRX = NULL;
    vTaskDelete(NULL);
}

//**************************************
//************* Init UARTs *************
//**************************************

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
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); //Configuro los pines del UART 
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, tamBUFFER * 2, tamBUFFER * 2, 20, &uart0_queue, 0));//config el UART por promedio de colas por queue UARTPINOHANGE deja el pin por defecto

    xTaskCreatePinnedToCore(TareaEventosUART0, "Tarea_para_UART0", 1024 * 5, NULL, 12, NULL, 1);
}

void app_main()
{
    initUART0();
}

//**************************************
//************* Funciones **************
//**************************************

void protocolo1Serial(uint8_t *ByteArray, uint16_t Length)
{

    //uart_write_bytes(UART_NUM_0, (const char*) ByteArray, Length);


    uint8_t estado = ByteArray[0]; // maquina de estado

    switch (estado)
    {
    case 0:

        break;

    case 1:

        break;
    case 2:

        break;

    case 3:

        break;
    }
}
