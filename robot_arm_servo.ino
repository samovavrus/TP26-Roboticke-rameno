#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "src\Control\Control.h"
#include "src\UI\ui_screens.h"
#include "src\DistanceSensor\RangingSensor.h"
#include "src\Trajectory\Trajectory.h"

TaskHandle_t HandleTaskUI;
TaskHandle_t HandleTaskSensor;


void setup(void) {

  Serial.begin(250000);
  Wire.begin();

  gUiToControlQueue = xQueueCreate(1, sizeof(UiControlCommandMessage));
  if (gUiToControlQueue == NULL) {
    Serial.println("Failed to create UI->Control queue");
  }

  gControlToUiQueue = xQueueCreate(1, sizeof(UiControlStateMessage));
  if (gControlToUiQueue == NULL) {
    Serial.println("Failed to create Control->UI queue");
  }

  xTaskCreate(TaskControl,
              "Control",
              5000,
              NULL,
              tskIDLE_PRIORITY + 4,
              &HandleTaskControl);
  
  xTaskCreate(TaskTrajectory,
            "Trajectory",
            1500,
            NULL,
            tskIDLE_PRIORITY + 1,
            &HandleTaskTrajectory);

  xTaskCreate(TaskUI,
              "UI",
              1500,
              NULL,
              tskIDLE_PRIORITY + 2,
              &HandleTaskUI);


xTaskCreate(TaskSensor,
              "Sensor",
              1000,
              NULL,
              tskIDLE_PRIORITY + 1,
              &HandleTaskSensor);

vTaskStartScheduler();
}

void loop(void) {
}


