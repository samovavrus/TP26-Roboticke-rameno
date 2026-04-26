#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "src\Control\Control.h"
#include "src\UI\ui_screens.h"
#include "src\DistanceSensor\RangingSensor.h"


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
              tskIDLE_PRIORITY + 3,
              &HandleTaskSensor);

vTaskStartScheduler();
}

void loop(void) {
}

void TaskSensor(void* pvParameters) {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  
  const uint32_t Ts_ms = 50; 
  // I2C bus: SDA=PB4 (D5), SCL=PA8 (D7)
  TwoWire Wire3(PB4, PA8);
  Wire3.begin();
  Wire3.setClock(400000);
  RangingSensor rangingSensor(Wire3);
  if (!rangingSensor.init(Ts_ms, RangingSensor::DistanceMode::Short)) {
    Serial.println("RangingSensor init failed!");
    vTaskDelete(NULL);
    return;
  }

  if (rangingSensor.initSD(PE4)) {
    Serial.println("SD Card initialized for RangingSensor!");
    rangingSensor.enableLogging(true, true); // 1.param: zapne logovanie, 2.param: vymaze existujuci subor
  } else {
    Serial.println("SD Card init failed! Logging disabled.");
  }

  rangingSensor.startContinuous();
  Serial.println("RangingSensor initialized");

  while (1) {
    MeasurementData measurement = rangingSensor.read(x, y, z, roll, pitch, yaw);
    
 /*   if (measurement.valid) {
      Serial.print("Distance: ");
      Serial.print(measurement.distance_mm);
      Serial.print(" mm | Pose: (");
      Serial.print(measurement.pose.x, 3); Serial.print(", ");
      Serial.print(measurement.pose.y, 3); Serial.print(", ");
      Serial.print(measurement.pose.z, 3); Serial.println(")");
    }*/

    vTaskDelay(pdMS_TO_TICKS(Ts_ms));
  }

}

