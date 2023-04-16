#include <Arduino.h>

void Tarea1(void *pvParameters) // Esta es una tarea
{

  while (1) // Una tarea nunca regresará ni saldrá
  {
    printf("Hola Mundo Tarea1 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Tarea3(void *pvParameters) // Esta es una tarea
{

  while (1) // Una tarea nunca regresará ni saldrá
  {
    printf("Hola Mundo Tarea3 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Tarea2(void *pvParameters) // Esta es una tarea
{

  while (1) // Una tarea nunca regresará ni saldrá
  {
    printf("Hola Mundo Tarea2 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Tarea4(void *pvParameters) // Esta es una tarea
{

  while (1) // Una tarea nunca regresará ni saldrá
  {
    printf("Hola Mundo Tarea4 freeRTOS Estoy corriendo en el núcleo = %d\n\r", xPortGetCoreID());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{

  xTaskCreatePinnedToCore(Tarea1, //Nombre de la función que se ejecutará
                          "Task1", //Nomnbre arbitrario de la tarea
                          1024 * 2, //Tamaño de la memoria que ocupa la tarea
                          NULL, //puntero estatico nulo
                          1, //orden de la prioridad de la tarea (prioridad 1)
                          NULL, //puntero de tipo handle nulo
                          0); //Núcleo en que se ejecutará la tarea (0)

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
}

void loop()
{
  // put your main code here, to run repeatedly:
}