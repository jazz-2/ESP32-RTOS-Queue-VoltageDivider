// https://github.com/jazz-2
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>

#define ADC1_0 36
#define LED_internal LED_BUILTIN
#define Core (BaseType_t)1
#define Vin 3.3f

#define queueSensorLength 3

#define ADC1_0_resolution_bits 12
#define ADC1_0_range ((pow(2, ADC1_0_resolution_bits) - 1))
#define analogWrite_resolution_bits 10
#define analogWrite_range ((uint32_t)(pow(2, analogWrite_resolution_bits) - 1))

#define structSensorSize sizeof(struct Sensor)
#define PointerSize sizeof(int8_t *)
#define PointerAddressSize_bit uint32_t // typedef uint32_t PointerAddressSize_bit;

#define samplesAmount 4

static QueueHandle_t queueHandleSensorData = NULL;
static QueueHandle_t queueHandleSensorCalc = NULL;
static TaskHandle_t handleSensorRead = NULL;
static TaskHandle_t handleSensorCalculate = NULL;
static TaskHandle_t handleSensorAction = NULL;

struct Sensor
{
  float ADC_to_Volt_average;
  float to_PWM;
  uint16_t ADC_Value[samplesAmount];
};

void sensorRead(void *myPrameters)
{
  struct Sensor *sensor1 = (struct Sensor *)pvPortMalloc(structSensorSize);

  while (1)
  {

    // Serial.println((String) "sensor1: " + (PointerAddressSize_bit)sensor1);
    // Serial.println((String) "&sensor1: " + (PointerAddressSize_bit)&sensor1);

    for (uint8_t i = 0; i < samplesAmount; i++)
    {
      sensor1->ADC_Value[i] = analogRead(ADC1_0);
      vTaskDelay(pdMS_TO_TICKS(250)); // vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    if (queueHandleSensorData != NULL)
    {
      if (xQueueSend(queueHandleSensorData, (void *)&sensor1, (TickType_t)10) == pdPASS)
      {
        // Serial.println("xQueueSend queueHandleSensorData success");
      }
      else
      {
        Serial.println("xQueueSend queueHandleSensorData failure");
      }
    }
    else
    {
      Serial.println("handle is NULL");
    }

    // Serial.println((String) "eTaskGetState: " + eTaskGetState(handleSensorRead));
    eTaskState SensorReadState;
    SensorReadState = eTaskGetState(handleSensorCalculate);
    // Serial.println((String) "SensorReadState: " + SensorReadState);
    if (SensorReadState == eSuspended)
    {
      vTaskResume(handleSensorCalculate); // if we resume a task with a higher priority it will preempt the current task
    }
  }
  vPortFree(sensor1);
  vTaskDelete(NULL);
}
void sensorAction(void *myPrameters)
{
  float PWM_DutyCycle;
  while (1)
  {
    if (queueHandleSensorCalc != NULL)
    {
      if (xQueueReceive(queueHandleSensorCalc, &PWM_DutyCycle, 10) == pdPASS)
      {
        analogWrite(LED_internal, (int)PWM_DutyCycle);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
void sensorCalculate(void *myPrameters)
{
  // struct Sensor *sensor1_buf; //can be used if we do not display "Serial.println(sensor1_buf->ADC_to_Volt_average);" before receiving data from the queue

  struct Sensor *sensor1_buf = (struct Sensor *)pvPortMalloc(structSensorSize);
  *sensor1_buf = (struct Sensor){0}; // initialization all members to 0. It is needed if we print "Serial.println(sensor1_buf->ADC_to_Volt_average);" before receiving data from the queue

  // struct Sensor *sensor1_buf;
  // struct Sensor s1;
  // sensor1_buf = &s1;

  if (sensor1_buf == NULL)
  {
    Serial.println("allocation failure");
  }
  else
  {
    Serial.println("allocation success");
  }

  while (1)
  {
    // Serial.println((String) "sensor1_buf: " + (PointerAddressSize_bit)sensor1_buf);
    // Serial.println((String) "&sensor1_buf: " + (PointerAddressSize_bit)&sensor1_buf);

    if (queueHandleSensorData != NULL)
    {
      if (xQueueReceive(queueHandleSensorData, (void *)&sensor1_buf, (TickType_t)0) != pdPASS)
      {
        // Serial.println("queue is empty");
      }
      else
      {
        sensor1_buf->ADC_to_Volt_average = 0;
        for (uint8_t i = 0; i < samplesAmount; i++)
        {
          sensor1_buf->ADC_to_Volt_average += (sensor1_buf->ADC_Value[i] * Vin) / ADC1_0_range;
        }
        sensor1_buf->ADC_to_Volt_average = sensor1_buf->ADC_to_Volt_average / (float)samplesAmount;
        sensor1_buf->to_PWM = (sensor1_buf->ADC_to_Volt_average * analogWrite_range) / Vin;
        if (queueHandleSensorCalc != NULL)
        {
          if (xQueueSend(queueHandleSensorCalc, (void *)&sensor1_buf->to_PWM, (TickType_t)0) == pdPASS)
          {
            // Serial.println("xQueueSend queueHandleSensorCalc success");
          }
          else
          {
            Serial.println("xQueueSend queueHandleSensorCalc failure");
          }
        }
        else
        {
          Serial.println("handle is NULL");
        }
      }
      for (uint8_t i = 0; i < samplesAmount; i++)
      {
        Serial.println((String) "sensor1_buf->ADC_Value[" + i + "]: " + sensor1_buf->ADC_Value[i]);
      }
      Serial.println((String) "sensor1_buf->ADC_to_Volt_average: " + sensor1_buf->ADC_to_Volt_average);
      Serial.println((String) "sensor1_buf->to_PWM: " + sensor1_buf->to_PWM);
    }
    else
    {
      Serial.println("handle is NULL");
    }

    vTaskSuspend(NULL);
    // vTaskDelay(pdMS_TO_TICKS(500));
  }
  vPortFree(sensor1_buf);
  vTaskDelete(NULL);
}
void setup()
{
  Serial.begin(115200);

  analogReadResolution(ADC1_0_resolution_bits);
  analogSetWidth(ADC1_0_resolution_bits);

  analogWriteResolution(analogWrite_resolution_bits);
  
  Serial.println((String) "analogWrite_range: " + analogWrite_range);
  Serial.println((String) "PointerAddressSize_bit: " + PointerSize * 8);

  Serial.println((String) "PointerSize: " + PointerSize);
  Serial.println((String) "structSensorSize: " + structSensorSize);

  // Serial.println((String) "uxTaskGetStackHighWaterMark: " + uxTaskGetStackHighWaterMark(NULL));
  // Serial.println((String) "uxTaskGetNumberOfTasks: " + uxTaskGetNumberOfTasks());
  Serial.println((String) "uxTaskPriorityGet: " + uxTaskPriorityGet(NULL));

  queueHandleSensorData = xQueueCreate(queueSensorLength, PointerSize);
  queueHandleSensorCalc = xQueueCreate(queueSensorLength, sizeof(float));

  Serial.println((String) "uxQueueSpacesAvailable: " + uxQueueSpacesAvailable(queueHandleSensorData));

  xTaskCreatePinnedToCore(sensorRead,
                          "read sensor data",
                          configMINIMAL_STACK_SIZE * 3,
                          NULL,
                          2,
                          &handleSensorRead,
                          Core);
  xTaskCreatePinnedToCore(sensorCalculate,
                          "calculate sensor data",
                          configMINIMAL_STACK_SIZE * 3,
                          NULL,
                          1,
                          &handleSensorCalculate,
                          Core);
  xTaskCreatePinnedToCore(sensorAction,
                          "react to sensor data",
                          configMINIMAL_STACK_SIZE * 3,
                          NULL,
                          1,
                          &handleSensorAction,
                          Core);

  ////// vTaskStartScheduler() shouldn't be called in ESP-IDF
  vTaskDelete(NULL);
}

void loop()
{
}