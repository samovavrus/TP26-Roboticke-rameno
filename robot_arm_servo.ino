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

#include "Eigen/Dense"

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "ui_screens.h"
#include "src\LiquidMenu\LiquidCrystal_I2C.h"
#include "src\LiquidMenu\LiquidMenu.h"
#include "src\Button.h"
#include "src\Joystick.h"
#include "src\Keypad\Keypad.h"

#include "ServoActuator.h"
#include "robot.h"

/// @brief Task handle for main control task (kinematics and servo control)
TaskHandle_t HandleTaskControl;

/// @brief Task handle for UI/display task
TaskHandle_t HandleTaskUI;

/// @brief Task handle for sensor acquisition task
TaskHandle_t HandleTaskSensor;

/**
 * @brief Arduino setup() - Initializes hardware and creates FreeRTOS tasks
 * 
 * System initialization sequence:
 * 1. Serial communication at 250 kbps for debug output
 * 2. I2C bus initialization for peripheral communication
 * 3. Create three concurrent FreeRTOS tasks:
 *    - TaskControl: Main kinematics solver (5000 bytes stack, priority IDLE+4)
 *    - TaskUI: Display and user input (1500 bytes stack, priority IDLE+2)
 *    - TaskSensor: Distance sensor (1000 bytes stack, priority IDLE+3)
 * 4. Start FreeRTOS scheduler
 * 
 * @note This function does not return; control passes to vTaskStartScheduler().
 *       The Arduino loop() function is never executed in this FreeRTOS-based design.
 */
void setup(void) {

  Serial.begin(250000);
  Wire.begin();

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

/**
 * @brief Arduino loop() - Not used in FreeRTOS design
 * 
 * This function is never executed because control passes to vTaskStartScheduler()
 * in setup(). All application logic runs within FreeRTOS tasks instead.
 */
void loop(void) {
}


/**
 * @brief FreeRTOS task for user interface and LCD display management
 * @param pvParameters FreeRTOS task parameter (unused)
 * 
 * Displays robot arm state information on 16×2 I2C LCD display:
 * - Reads analog inputs simulating end-effector position data (X, Y, Z)
 * - Derives orientation (Roll, Pitch, Yaw) from position signals
 * - Monitors button A (pin 2) for screen switching (200 ms cycle)
 * - Alternates between two display modes:
 *   1. Screen 0: End-effector position (X, Y, Z Cartesian coordinates)
 *   2. Screen 1: End-effector orientation (Roll, Pitch, Yaw angles)
 * 
 * Hardware connections:
 * - LCD address: 0x27 (I2C, 16×2 character display)
 * - Button A: Pin 2 (INPUT_PULLUP, triggers display switch on falling edge)
 * - Analog X: A0 (simulates X position)
 * - Analog Y: A1 (simulates Y position)
 * 
 * Task behavior:
 * - Initializes LCD backlight and clears display
 * - Infinite loop: reads inputs, updates display, sleeps 200 ms
 * - Screen toggle via falling edge detection
 * 
 * @note Currently uses simulated analog values. In production, should receive
 *       actual end-effector pose from TaskControl via shared memory or queue.
 *       Consider protecting shared data with mutex if TaskControl updates values.
 * 
 * @todo Replace simulated analog values with actual forward kinematics results
 * @todo Add FreeRTOS queue or shared data structure for pose communication
 */
void TaskUI(void* pvParameters) {

  LiquidCrystal_I2C lcd(0x27, 16, 2);

  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();

  pinMode(2, INPUT_PULLUP); // A button

  int screen = 0;
  bool lastButtonState = HIGH;

  while (1) {

    // --- simulované hodnoty (zatiaľ)
    float x = analogRead(A0) / 10.0;
    float y = analogRead(A1) / 10.0;
    float z = (x+y)/2;

    float roll  = x/5;
    float pitch = y/5;
    float yaw   = z/5;

    // --- čítanie tlačidla A
    bool currentState = digitalRead(2);

    // detekcia stlačenia (falling edge)
    if (lastButtonState == HIGH && currentState == LOW) {
      screen = !screen;   // prepni obrazovku
    }

    lastButtonState = currentState;

    // --- vykreslenie
    if (screen == 0) {
      drawXYZ(lcd, x, y, z);
    } else {
      drawRPY(lcd, roll, pitch, yaw);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}


/**
 * @brief FreeRTOS task for main robot kinematics and servo control
 * @param pvParameters FreeRTOS task parameter (unused)
 * 
 * Main control task implementing 6-DOF robot arm kinematics and servo actuation:
 * 
 * **Hardware Configuration:**
 * - I2C Bus 2 (Wire2): PF0 (SDA), PF1 (SCL), 400 kHz clock
 * - PCA9685 PWM controller: Controls 6 servo motors (channels 0-5)
 * - Servo frequency: 50 Hz (standard for hobby servos)
 * 
 * **Kinematic Chain (6-DOF Arm):**
 * - Joint 1: Rotation Z (base/waist) - 0.094 m link
 * - Joint 2: Rotation X (shoulder) - 0.105 m link
 * - Joint 3: Rotation X (elbow) - 0.147 m link
 * - Joint 4: Rotation X (forearm) - 0.097 m link offset
 * - Joint 5: Rotation Z (wrist pitch) - 0.0215 m offset
 * - Joint 6: Rotation Y (end-effector) - 0.070 m to gripper
 * - Total reach: ~0.455 m at full extension
 * 
 * **Servo Parameters (per joint):**
 * Each joint has calibrated parameters:
 * - Angle limits (min/max in radians)
 * - PWM gain factor (radians to degrees conversion)
 * - Angle offset (systematic calibration)
 * 
 * **Current Implementation:**
 * - Initializes Eigen-based kinematics solver with RobotKinematics class
 * - Performs forward kinematics test at zero joint angles
 * - Prints Jacobian matrix (3×6 position derivatives) to serial
 * - Tests inverse kinematics solver toward target position [0.15, 0, 0.20] m
 * - Validates IK convergence with tolerances:
 *   - Position: 0.1 mm
 *   - Orientation: 1e-3 rad
 *   - Levenberg-Marquardt damping: 0.01
 *   - Max iterations: 50
 * 
 * **Task Flow:**
 * 1. Initialize Wire2 I2C bus and set clock speed
 * 2. Configure servo parameters for 6 joints
 * 3. Create RobotServoController and initialize PCA9685
 * 4. Define kinematic structure (rotation axes and link vectors)
 * 5. Run forward kinematics test (theta = 0)
 * 6. Compute and print Jacobian
 * 7. Run inverse kinematics test to target position
 * 8. Infinite loop with 10 ms delay (allows other tasks to run)
 * 
 * **Output (Serial at 250 kbps):**
 * - Forward kinematics result: [x, y, z] at theta=0
 * - 3×6 Jacobian matrix (position derivatives)
 * - IK convergence status ("úspešne" = success / "zlyhala" = failed)
 * - Converged joint angles in radians (if successful)
 * 
 * @note This task currently runs kinematics tests once and then idles.
 *       Production code should implement a control loop reading desired poses
 *       and commanding servo positions continuously.
 * 
 * @todo Implement real-time control loop
 * @todo Add safety checks and joint limit enforcement
 * @todo Integrate actual position feedback from servos
 * @todo Add task communication (queue/mutex) with TaskUI for pose visualization
 * @todo Implement trajectory planning and motion interpolation
 */
void TaskControl(void* pvParameters) {

  TwoWire Wire2(PF0, PF1);
  Wire2.begin();
  Wire2.setClock(400000);

  std::vector<actuator_parameters> servo_parameters;
  servo_parameters.resize(6);
  servo_parameters[0] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0 };
  servo_parameters[1] = { -5.0 / 10.0 * PI, 5.2 / 10.0 * PI, 2.0 / 3.0, 0.015 };
  servo_parameters[2] = { -8.2 / 10.0 * PI, 5.0 / 10.0 * PI, -2.0 / 3.0, 0.12 };
  servo_parameters[3] = { -7.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, -0.2 };
  servo_parameters[4] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0.0 };
  servo_parameters[5] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, -2.0 / 3.0, 0.0 };

  RobotServoController servo_controller(Wire2, servo_parameters);
  servo_controller.init();

  // --- Definícia kinematiky robota ---
  // Rotačné osi každého kĺbu (uprav podľa konštrukcie robota)
  static const RotationZ R1;  // kĺb 1 – otáča v základni
  static const RotationX R2;  // kĺb 2 – zdvíha rameno
  static const RotationX R3;  // kĺb 3 – ohýba rameno
  static const RotationX R4;  // kĺb 4 – rotuje predlaktie
  static const RotationZ R5;  // kĺb 5 – ohýba zápästie
  static const RotationY R6;  // kĺb 6 – rotuje efektor

  const std::array<const RotationMatrix*, 6> joint_axes = { &R1, &R2, &R3, &R4, &R5, &R6 };

  // 3D Translačné vektory podľa výkresu. 
  const std::array<Matrix<3, 1>, 6> link_translations = {
    (Matrix<3,1>() << 0.0f, 0.0f, 0.094f).finished(),  // rotacia okolo Z (servo1) a rotacia okolo X (servo2)
    (Matrix<3,1>() << 0.0f, 0.0f, 0.105f).finished(),  // rotacia okolo X (servo3)
    (Matrix<3,1>() << 0.0f, 0.0f, 0.147f).finished(),  // rotacia okolo X (servo4)
    (Matrix<3,1>() << 0.0f, 0.009f, 0.097f).finished(),  // rotacia okolo Z (servo5)
    // offset zápästia. Ak je mimo osi do boku v smere X, zmeň 0.0 na 0.0125 atd...
    (Matrix<3,1>() << 0.015f, -0.0215f, 0.0f).finished(), // rotacia okolo X (servo6)
    (Matrix<3,1>() << 0.0f, 0.0f, 0.070f).finished()   // efektor
  };

  // Vytvorenie objektu kinematiky - ZMENIŤ link_lengths na link_translations!
  Robot::RobotKinematics<6> kinematics(link_translations, joint_axes);

  // --- Test priamej kinematiky ---
  Matrix<6,1> theta = Matrix<6,1>::Zero();  // všetky kĺby v nulovej polohe

  auto result = kinematics.forwardKinematics(theta);
  Matrix<3,1> pos = result.first;
  Matrix<3,3> rot = result.second;

  Serial.print("FK x="); Serial.println(pos(0), 4);
  Serial.print("FK y="); Serial.println(pos(1), 4);
  Serial.print("FK z="); Serial.println(pos(2), 4);
  // Očakávané pri theta=0: x=0, y=0, z=sum(L) = 0.455 m
  auto J = kinematics.getJacobian(theta);   // 3x6 pozicny Jacobian

  Serial.println("J:");
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      Serial.print(J(r, c), 6);
      if (c < 5) Serial.print(" ");
    }
    Serial.println();
  }

    // 1. Zadefinovanie cieľovej polohy pr (napríklad [X, Y, Z] v metroch)
  Matrix<3, 1> target_pos;
  target_pos << 0.15f,  // X
                0.0f,   // Y
                0.20f;  // Z

  // 2. Zadefinovanie cieľovej orientácie Rr (napríklad len identita = rovnaká orientácia ako v nulovej polohe)
  Matrix<3, 3> target_rot = Matrix<3, 3>::Identity(); 

  // 3. Počiatočný odhad kĺbov 'theta'
  // Najlepšie je sem dať AKTUÁLNE natočenie (teraz pre test dáme samé nuly)
  Matrix<6, 1> current_theta = Matrix<6, 1>::Zero();

  // 4. Pripravenie parametrov pre solver
  float tol_pos = 1e-4f;  // Tolerancia polohy (napr. 0.1 mm)
  float tol_ori = 1e-3f;  // Tolerancia orientácie 
  float lambda = 0.01f;   // Regularizačný/tlmiaci faktor (damping factor)
  int max_iter = 50;      // Maximálny počet iterácií

  // 5. Samotné zavolanie funkcie
  bool success = kinematics.SolveIK(
      target_pos, 
      target_rot, 
      current_theta, // Pozor, táto premenná sa vo vnútri funkcie upraví na výsledok!
      tol_pos, 
      tol_ori, 
      lambda, 
      max_iter
  );

  // 6. Kontrola výsledku
  if (success) {
      Serial.println("IK úspešne našla riešenie!");
      Serial.println("Nové uhly kĺbov (v radiánoch):");
      for(int i = 0; i < 6; i++) {
          Serial.println(current_theta(i), 4);
      }
  } else {
      Serial.println("IK zlyhala / nekonvergovala.");
  }

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#include "src\VL53L1X\VL53L1X.h"

/**
 * @brief FreeRTOS task for distance sensor data acquisition
 * @param pvParameters FreeRTOS task parameter (unused)
 * 
 * Manages a VL53L1X time-of-flight (ToF) distance sensor for obstacle detection
 * and end-effector proximity sensing:
 * 
 * **Hardware Configuration:**
 * - I2C Bus 3 (Wire3): PB4 (SDA), PA8 (SCL), 400 kHz clock
 * - Sensor: ST Microelectronics VL53L1X
 * - Measurement range: Short range mode (up to 1.3 m typical)
 * 
 * **Sensor Configuration:**
 * - Sampling period (Ts): 0.1 seconds = 100 ms
 * - Measurement timing budget: 80% of Ts = 80 ms
 *   (Time allowed for single ToF measurement)
 * - Timeout: 150% of Ts = 150 ms
 *   (Watchdog timer to prevent hangs)
 * - Operating mode: Continuous ranging at 10 Hz
 * 
 * **Initialization Sequence:**
 * 1. Create Wire3 I2C instance on specific pins
 * 2. Initialize I2C bus at 400 kHz
 * 3. Initialize VL53L1X sensor communication
 * 4. Configure measurement timing (80 ms per measurement)
 * 5. Set to short-range mode for faster response
 * 6. Start continuous ranging mode (automatic 10 Hz sampling)
 * 
 * **Current Implementation:**
 * - Initializes sensor and timing configuration
 * - Enters infinite loop with 10 ms sleep (allows other tasks to run)
 * - Sensor continuously acquires distance data in background
 * 
 * **Usage:**
 * - Sensor data can be read via VL53L1X API calls
 * - Output available via standard VL53L1X getDistance() method
 * - Consider: velocity measurement, collision avoidance, object tracking
 * 
 * @note Current code only initializes sensor; no data reading or processing.
 *       This task primarily yields CPU time to other tasks.
 * 
 * @todo Implement actual distance reading from sensor
 * @todo Add obstacle detection logic and collision avoidance
 * @todo Create FreeRTOS queue to communicate sensor data to TaskControl
 * @todo Add sensor error handling and status monitoring
 * @todo Consider integrating with motion planning for safe operation
 */
void TaskSensor(void* pvParameters) {

  const float Ts = 0.1;
  TwoWire Wire3(PB4, PA8);
  Wire3.begin();
  Wire3.setClock(400000);
  VL53L1X sensor_TOF;
  sensor_TOF.setBus(&Wire3);

  sensor_TOF.init();
  sensor_TOF.setMeasurementTimingBudget((uint32_t)(0.8 * Ts * 1000.0));
  sensor_TOF.setDistanceMode(VL53L1X::Short);
  sensor_TOF.setTimeout((uint16_t)(Ts * 1.5 * 1000.0));
  sensor_TOF.startContinuous((uint32_t)(Ts * 1000.0));

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

}

