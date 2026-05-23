/**
 * @file robot_arm_servo.ino
 * @brief Main firmware for STM32-based robot arm control system
 * 
 * Embedded control system for a 6-DOF robotic manipulator arm using:
 * - **Kinematics**: Forward and inverse kinematics with Levenberg-Marquardt solver
 * - **Real-time OS**: FreeRTOS for concurrent task management
 * - **Hardware**: STM32 microcontroller with I2C peripherals
 * - **Actuation**: PCA9685 PWM driver controlling servo motors
 * - **Sensing**: VL53L1X time-of-flight distance sensor
 * - **UI**: 16×2 LCD display for real-time monitoring
 * 
 * System architecture:
 * - **TaskControl** (Priority: IDLE+4): Main robot kinematics and servo control
 * - **TaskUI** (Priority: IDLE+2): User interface and display management
 * - **TaskSensor** (Priority: IDLE+3): Distance sensor data acquisition
 * 
 * All tasks run concurrently under FreeRTOS scheduler with preemptive scheduling.
 */

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

/// @brief Task handle for sensor acquisition task
TaskHandle_t HandleTaskSensor;

// Vytvorenie globálnej premennej fronty
QueueHandle_t gUiToTrajectoryQueue = NULL;

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

    // Fronta s dĺžkou 1 správa s veľkosťou jedného uint8_t
  gUiToTrajectoryQueue = xQueueCreate(1, sizeof(uint8_t));
  if (gUiToTrajectoryQueue == NULL) {
    Serial.println("Failed to create UI->Trajectory queue");
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

/**
 * @brief Arduino loop() - Not used in FreeRTOS design
 * 
 * This function is never executed because control passes to vTaskStartScheduler()
 * in setup(). All application logic runs within FreeRTOS tasks instead.
 */
void loop(void) {
}


