#include "uarts.h"
#include "tareas.h"

void app_main()
{

  initUART0();

  xTaskCreatePinnedToCore(TareaEntradaDatos, "Tarea_para_entrada1", 1024 * 2, NULL, 3, NULL, 1);

  xTaskCreatePinnedToCore(TareaSetCoils, "Tarea_Salida_Binarias", 1024 * 2, NULL, 2, NULL, 1);
}