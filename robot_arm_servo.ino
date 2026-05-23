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

/// @brief Task handle for main control task (kinematics and servo control)
TaskHandle_t HandleTaskControl;

/// @brief Task handle for UI/display task
TaskHandle_t HandleTaskUI;

/// @brief Task handle for sensor acquisition task
TaskHandle_t HandleTaskSensor;

/// @brief Task handle for trajectory generation and planning
TaskHandle_t HandleTaskTrajectory;

/// @brief Queue for communication from UI task to Trajectory task
/// @details Used to send trajectory commands (uint8_t format) from the user interface
/// to the trajectory generation and planning module. Queue size: 1 message.
QueueHandle_t gUiToTrajectoryQueue = NULL;

/**
 * @brief Arduino setup() - Initializes hardware and creates FreeRTOS tasks
 * 
 * System initialization sequence:
 * 1. Serial communication at 250 kbps for debug output
 * 2. I2C bus initialization for peripheral communication
 * 3. Create inter-task communication queues:
 *    - gUiToControlQueue: UI → Control task commands
 *    - gControlToUiQueue: Control → UI task state updates
 *    - gUiToTrajectoryQueue: UI → Trajectory task commands
 * 4. Create four concurrent FreeRTOS tasks:
 *    - TaskControl: Main kinematics solver (5000 bytes stack, priority IDLE+4)
 *    - TaskTrajectory: Trajectory planning (1500 bytes stack, priority IDLE+1)
 *    - TaskUI: Display and user input (1500 bytes stack, priority IDLE+2)
 *    - TaskSensor: Distance sensor (1000 bytes stack, priority IDLE+1)
 * 5. Start FreeRTOS scheduler
 * 
 * @note This function does not return; control passes to vTaskStartScheduler().
 *       The Arduino loop() function is never executed in this FreeRTOS-based design.
 */
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


