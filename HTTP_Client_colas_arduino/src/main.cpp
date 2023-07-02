#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdio.h>
#include "driver/i2c.h" //Incluimos la biblioteca del controlador I2C del ESP32 para comunicarnos con el sensor MPU6050.
#include "def_sensores.h"

TaskHandle_t xHandle_http_task = NULL;
TaskHandle_t xHandle_entrada_datos = NULL;
TimerHandle_t xTimer1, xTimer2;
int timerId_1 = 1;
int timerId_2 = 1;
int intervalo = 100;  // constante de tiempo para ser llamado el timer1
int intervalo2 = 150; // constante de tiempo para ser llamado el timer2
static const char *TAG2 = "Estado: ";
Dato sensor;

QueueHandle_t queue;

uint8_t buffer_distancia_2[14]; // Creamos un arreglo para almacenar los datos leídos del MPU6050.
int16_t ax, ay, az, gx, gy, gz; // Enteros de 16 bits se utilizan para almacenar los valores leidos del acelerómetro y el giroscopio.

#define TRIGGER_PIN (gpio_num_t)12
#define ECHO_PIN (gpio_num_t)13
#define PIR_PIN (gpio_num_t)14
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO (gpio_num_t)21 // PIN SDA del ESP32
#define I2C_SCL_GPIO (gpio_num_t)22 // PIN SCL del ESP32
#define MPU6050_ADDR 0x68           // Direccion del sensor MPU6050

void mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void i2c_master_init();
esp_err_t set_timer1(void);
esp_err_t set_timer2(void);

//**************************************
//*************** TASKs ****************
//**************************************

void TaskEntradaDatos(void *pvParameters)
{

  if (queue == 0)
  {
    printf("Failed to create queue= %p\n", queue);
  }

  while (1)
  {
    if (gpio_get_level(PIR_PIN) == 1)
    {
      printf("Movimiento detectado\n");
      sensor.PIR = 1;
      xQueueSend(queue, &sensor, pdMS_TO_TICKS(100));
    }
    else
    {
      sensor.PIR = 0;
      xQueueSend(queue, &sensor, pdMS_TO_TICKS(100));
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskHTTP(void *pvParameters)
{

  // Scada Vemetris en Digital Ocean
  String ScadaVemetris = "http://137.184.178.17:21486/httpds?__device=http_roberto";

  Dato rxsensor;

  while (1)
  {

    if (xQueueReceive(queue, &rxsensor, pdMS_TO_TICKS(100)) == pdPASS)
    {

      HTTPClient http;

      String dato1 = String(rxsensor.ULTRA);

      String dato2 = String(rxsensor.PIR);

      String dato3 = String(rxsensor.acelx);

      String dato4 = String(rxsensor.acely);

      String dato5 = String(rxsensor.acelz);

      String dato6 = String(rxsensor.gcelx);

      String dato7 = String(rxsensor.gcely);

      String dato8 = String(rxsensor.gcelz);

      String Trama = ScadaVemetris + "&rssi=" + WiFi.RSSI() + "&dato1=" + dato1 + "&dato2=" + dato2 + "&dato3=" + dato3 + "&dato4=" + dato4 + "&dato5=" + dato5 + "&dato6=" + dato6 + "&dato7=" + dato7 + "&dato8=" + dato8;

      printf("Distancia %s \n", dato1);
      printf("Estado del PIR %s \n", dato2);
      printf("Acelerometro en x %s \n", dato3);
      printf("Acelerometro en y %s \n", dato4);
      printf("Acelerometro en z %s \n", dato5);
      printf("Giroscopio en x %s \n", dato6);
      printf("Giroscopio en y %s \n", dato7);
      printf("Giroscopio en z %s \n", dato8);

      Serial.println(Trama);
      http.begin(Trama);         // Iniciar conexión
      int httpCode = http.GET(); // Realizar petición
      if (httpCode > 0)
      {
        String payload = http.getString(); // Obtener respuesta
        Serial.println(httpCode);          // Si el codigo es 200, se realizo bien
        Serial.println(payload);           // Mostrar respuesta por serial
      }
      else
      {
        Serial.println("Error enviando la trama");
      }
      http.end(); // Se libera el cliente

      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void initWiFi(void)
{

  const char *ssid = "TuPapiBello";
  const char *password = "*90mH231";

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.println("Configurando Red Wi-Fi");
  }
}

void mpu6050_init()
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();                           // Creamos un mango para una lista de comandos I2C.
  i2c_master_start(cmd);                                                  // Enviamos la señal de inicio de la comunicación I2C.
  i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true); // Escribimos la dirección del sensor MPU6050 en el bus I2C en modo escritura.
  i2c_master_write_byte(cmd, 0x6B, true);                                 // Escribimos en el registro de control de energía del MPU6050 para encender el sensor.
  i2c_master_write_byte(cmd, 0x00, true);                                 // Escribimos en el registro de configuración del MPU6050 para configurar el sensor en modo de medición.
  i2c_master_stop(cmd);                                                   // Enviamos la señal de parada de la comunicación I2C.
  i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);         // Ejecutamos la lista de comandos I2C.
  i2c_cmd_link_delete(cmd);                                               // Borramos el mango de la lista de comandos I2C.
}

// Función para leer los valores del sensor MPU6050.

void mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();                           // Declara una variable de tipo i2c_cmd_handle_t llamada cmd y la inicializa con un nuevo comando I2C. Esta variable se utiliza para construir el comando I2C que se enviará al sensor.
  i2c_master_start(cmd);                                                  // Envía un comando de inicio al bus I2C utilizando el comando I2C almacenado en la variable cmd. Este comando indica al sensor que se inicie una nueva transacción.
  i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true); // Envía la dirección del sensor MPU6050 al bus I2C utilizando el comando I2C almacenado en la variable cmd. El parámetro true indica que se debe enviar un comando ACK (confirmación de recepción) después de enviar la dirección.
  i2c_master_write_byte(cmd, 0x3B, true);                                 // Envía la dirección del registro inicial que se va a leer en el sensor MPU6050 utilizando el comando I2C almacenado en la variable cmd. El parámetro true indica que se debe enviar un comando ACK después de enviar la dirección.
  i2c_master_start(cmd);                                                  // Envía un segundo comando de inicio al bus I2C utilizando el comando I2C almacenado en la variable cmd. Este comando indica al sensor que se iniciará una nueva transacción de lectura.
  i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, true);  // Envía la dirección del sensor MPU6050 al bus I2C utilizando el comando I2C almacenado en la variable cmd, pero esta vez con el bit de lectura activado. El parámetro true indica que se debe enviar un comando ACK después de enviar la dirección.
  i2c_master_read(cmd, buffer_distancia_2, 14, I2C_MASTER_LAST_NACK);     // Lee 14 bytes de datos del sensor MPU6050 utilizando el comando I2C almacenado en la variable cmd y los almacena en la matriz buffer_distancia_2. El parámetro I2C_MASTER_LAST_NACK indica que se debe enviar un comando NACK (Generar confirmación de No-recepción,) después de leer los datos.
  i2c_master_stop(cmd);                                                   // Envía un comando de parada al bus I2C utilizando el comando I2C almacenado en la variable cmd. Este comando indica al sensor que se ha completado la transacción.
  i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);         // Envía el comando I2C almacenado en la variable cmd al puerto I2C especificado (I2C_PORT) utilizando la función i2c_master_cmd_begin(). Con tiempo de espera maximo de  1seg para la transacción
  i2c_cmd_link_delete(cmd);                                               // Libera la memoria utilizada por el comando I2C almacenado en la variable cmd.
  *ax = buffer_distancia_2[0] << 8 | buffer_distancia_2[1];               // Asigna el valor de los primeros dos bytes de la matriz buffer (que corresponden a los valores de aceleración en el eje X) a la variable apuntada por ax. Los bytes se combinan utilizando una operación OR bit a bit.
  *ay = buffer_distancia_2[2] << 8 | buffer_distancia_2[3];               // Asigna el valor de los siguientes dos bytes de la matriz buffer (que corresponden a los valores de aceleración en el eje Y) a la variable apuntada por ay. Los bytes se combinan utilizando una operación OR bit a bit.
  *az = buffer_distancia_2[4] << 8 | buffer_distancia_2[5];               // Asigna el valor de los siguientes dos bytes de la matriz buffer (que corresponden a los valores de aceleración en el eje Z) a la variable apuntada por az. Los bytes se combinan utilizando una operación OR bit a bit.
  *gx = buffer_distancia_2[8] << 8 | buffer_distancia_2[9];               // Asigna el valor de los siguientes dos bytes de la matriz buffer (que corresponden a los valores de velocidad angular en el eje X) a la variable apuntada por gx. Los bytes se combinan utilizando una operación OR bit a bit.
  *gy = buffer_distancia_2[10] << 8 | buffer_distancia_2[11];             // Asigna el valor de los siguientes dos bytes de la matriz buffer (que corresponden a los valores de velocidad angular en el eje Y) a la variable apuntada por gy. Los bytes se combinan utilizando una operación OR bit a bit.
  *gz = buffer_distancia_2[12] << 8 | buffer_distancia_2[13];             // Asigna el valor de los últimos dos bytes de la matriz buffer (que corresponden a los valores de velocidad angular en el eje Z) a la variable apuntada por gz. Los bytes se combinan utilizando una operación OR bit a bit.
}

void i2c_master_init()
{
  i2c_config_t conf;                                // Se crea una estructura de configuración para el controlador I2C
  conf.mode = I2C_MODE_MASTER;                      // Configuramos el controlador I2C como modo maestro
  conf.sda_io_num = I2C_SDA_GPIO;                   // Configuramos el PIN SDA para la linea de datos
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;          // Habilitamos la resistencia pull-up interna para la línea de datos.
  conf.scl_io_num = I2C_SCL_GPIO;                   // Configuramos el pin GPIO para la línea de reloj (SCL).
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;          // Habilitamos la resistencia pull-up interna para la línea de reloj.
  conf.master.clk_speed = 100000;                   // Configuramos la velocidad de reloj del I2C en 100 kHz.
  i2c_param_config(I2C_PORT, &conf);                // Configuramos los parámetros de la interfaz I2C.
  i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0); // Instalamos el controlador I2C.
}

void setup_pin()
{
  gpio_reset_pin(TRIGGER_PIN);                       // Se resetea cualquier valor en el pin GPIO_12
  gpio_reset_pin(ECHO_PIN);                          // Se resetea cualquier valor del pin GPIO_13
  gpio_reset_pin(PIR_PIN);                           // Se resetea cualquier valor del pin GPIO_14
  gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT); // Direccion del pin trigger del ultra como salida
  gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);     // Direccion del pin GPIO_13 en modo de entrada (Eco del sensor ultra)
  gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);      // Direccion del PIN del PIR EN MODO ENTRADA
}

void vTimerCallback1(TimerHandle_t pxTimer)
{
  // Generar un pulso de 10 us en el pin de trigger
  gpio_set_level(TRIGGER_PIN, 1);
  esp_rom_delay_us(10);
  gpio_set_level(TRIGGER_PIN, 0);

  // Medir el tiempo que tarda el eco en regresar
  uint32_t start_time = 0;
  uint32_t end_time = 0;
  while (gpio_get_level(ECHO_PIN) == 0)
  {
    start_time = esp_timer_get_time();
  }
  while (gpio_get_level(ECHO_PIN) == 1)
  {
    end_time = esp_timer_get_time();
  }

  // Calcular la distancia en centímetros
  uint32_t pulse_duration = end_time - start_time;
  float distance = pulse_duration / 58.0f;

  // ESP_LOGI(TAG, "Distancia: %.2f cm", distance);

  sensor.ULTRA = distance;

  xQueueSend(queue, &sensor, pdMS_TO_TICKS(100));
}

void vTimerCallback2(TimerHandle_t pxTimer)
{
  mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz); // Lee los valores del acelerómetro y el giroscopio y los almacena en las variables ax, ay, az, gx, gy y gz.
  sensor.acelx = ax;
  sensor.acely = ay;
  sensor.acelz = az;
  sensor.gcelx = gx;
  sensor.gcely = gy;
  sensor.gcelz = gz;

  printf("Acelerómetro: X=%d, Y=%d, Z=%d\n", ax, ay, az); // Imprime valores del acelerómetro en consola
  printf("Giroscopio: X=%d, Y=%d, Z=%d\n", gx, gy, gz);   // Imprime valores del giroscopio en consola

  xQueueSend(queue, &sensor, pdMS_TO_TICKS(100));
}

esp_err_t set_timer1(void)
{
  ESP_LOGI(TAG2, "Inicia la configuración del temporizador 1");
  xTimer1 = xTimerCreate("Timer1",                   // Just a text name, not used by the kernel.
                         (pdMS_TO_TICKS(intervalo)), // The timer period in ticks.
                         pdTRUE,                     // The timers will auto-reload themselves when they expire.
                         (void *)timerId_1,          // Assign each timer a unique id equal to its array index.
                         vTimerCallback1             // Each timer calls the same callback when it expires.
  );

  if (xTimer1 == NULL)
  {
    // The timer was not created.
    ESP_LOGI(TAG2, "El temporizador no se ejecutó");
  }
  else
  {
    if (xTimerStart(xTimer1, 0) != pdPASS)
    {
      // The timer could not be set into the Active state.
      ESP_LOGI(TAG2, "El temporizador no se pudo establecer en el estado activo");
    }
  }

  return ESP_OK;
}

esp_err_t set_timer2(void)
{
  ESP_LOGI(TAG2, "Inicia la configuración del temporizador 1");
  xTimer2 = xTimerCreate("Timer2",                    // Just a text name, not used by the kernel.
                         (pdMS_TO_TICKS(intervalo2)), // The timer period in ticks.
                         pdTRUE,                      // The timers will auto-reload themselves when they expire.
                         (void *)timerId_2,           // Assign each timer a unique id equal to its array index.
                         vTimerCallback2              // Each timer calls the same callback when it expires.
  );

  if (xTimer2 == NULL)
  {
    // The timer was not created.
    ESP_LOGI(TAG2, "El temporizador no se ejecutó");
  }
  else
  {
    if (xTimerStart(xTimer2, 0) != pdPASS)
    {
      // The timer could not be set into the Active state.
      ESP_LOGI(TAG2, "El temporizador no se pudo establecer en el estado activo");
    }
  }

  return ESP_OK;
}

void setup()
{

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  setup_pin(); // Se inicia la config de los pines GPIO

  i2c_master_init(); // Inicializa el bus I2C

  mpu6050_init(); // Inicializa el sensor MPU6050

  initWiFi();

  queue = xQueueCreate(10, sizeof(Dato));

  set_timer1();
  set_timer2();

  xTaskCreatePinnedToCore(TaskEntradaDatos, "EntradaDatos", 4096, NULL, 2, &xHandle_entrada_datos, 1);

  xTaskCreatePinnedToCore(TaskHTTP, "HTTPcliente", 4096, NULL, 4, &xHandle_http_task, 1);
}

void loop()
{
}