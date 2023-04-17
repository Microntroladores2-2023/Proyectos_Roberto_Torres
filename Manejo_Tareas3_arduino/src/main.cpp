#include "tareas.h"

TaskHandle_t xHandle;

void setup()
{

  xTaskCreatePinnedToCore(Boton,
                          "Boton",
                          1024 * 2,
                          NULL,
                          1,
                          NULL,
                          0);

  xTaskCreatePinnedToCore(Blink,
                          "Parpadeo",
                          1024 * 2,
                          NULL,
                          1,
                          NULL,
                          0);
}

void loop()
{
  // put your main code here, to run repeatedly:
}