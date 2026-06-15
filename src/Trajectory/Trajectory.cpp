/**
 * @file Trajectory.cpp
 * @brief Trajectory generation task implementation.
 */
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "Trajectory.h"

/// @brief FreeRTOS task handle for the trajectory task.
TaskHandle_t HandleTaskTrajectory;

// Globálna cieľová póza pre Control task
/// @brief Global trajectory target pose shared with Control task.
volatile TrajectoryTargetPose gTrajectoryTargetPose = {0};

// Riadiace flagy pre spustenie/zastavenie trajektórie z UI
static volatile uint8_t gTrajectoryStartRequested = 0;
static volatile uint8_t gTrajectoryStopRequested  = 0;
static volatile uint8_t gTrajectoryRunning        = 0;

#define DEG2RAD(x) ((x) * 0.01745329252f)

/**
 * @brief Trajectory waypoint with pose and segment duration.
 */
struct Waypoint {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  uint32_t duration_ms;
};

/**
 * @brief Linear interpolation helper.
 * @param a Start value.
 * @param b End value.
 * @param t Interpolation factor in [0, 1].
 * @return Interpolated value.
 */
static float lerp(float a, float b, float t) {
  return a + t * (b - a);
}

/**
 * @brief Update the shared trajectory pose in a critical section.
 * @param x Position X in meters.
 * @param y Position Y in meters.
 * @param z Position Z in meters.
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw Yaw angle in radians.
 */
static void setTrajectoryPose(float x, float y, float z,
                              float roll, float pitch, float yaw) {
  taskENTER_CRITICAL();

  gTrajectoryTargetPose.x = x;
  gTrajectoryTargetPose.y = y;
  gTrajectoryTargetPose.z = z;

  gTrajectoryTargetPose.roll  = roll;
  gTrajectoryTargetPose.pitch = pitch;
  gTrajectoryTargetPose.yaw   = yaw;

  // valid nastavujeme až nakoniec, aby Control vedel,
  // že dáta trajektórie sú pripravené
  gTrajectoryTargetPose.valid = 1;

  taskEXIT_CRITICAL();
}

/**
 * @brief Request the trajectory task to start scanning.
 */
void Trajectory_StartScan(void) {
  gTrajectoryStartRequested = 1;
  gTrajectoryStopRequested = 0;
}

/**
 * @brief Request the trajectory task to stop scanning.
 */
void Trajectory_StopScan(void) {
  gTrajectoryStopRequested = 1;
}

/**
 * @brief Return whether the trajectory task is currently running.
 */
bool Trajectory_IsRunning(void) {
  return gTrajectoryRunning != 0;
}

/**
 * @brief FreeRTOS task for trajectory planning and pose streaming.
 * @param pvParameters Unused task parameter.
 */
void TaskTrajectory(void* pvParameters) {
  const uint32_t Ts_ms = 50;


  Waypoint path[500];
  int N = 0;
  
  // Počiatočné body
  path[N++] = { 0.015f, -0.388f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(0.0), 1000 };
  path[N++] = { 0.015f, -0.333f, 0.285f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(0.0), 1000 };
  path[N++] = { 0.015f, -0.340f, 0.242f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(0.0), 1000 };

  // Skenovanie (zig-zag): Postupujeme v roll od 90 do 0 (alebo koľko potrebujete, zatial po 80)
  int direction = 1; // 1 znamená pohyb z yaw +50 na -50, -1 z -50 na +50
  for (int r = 90; r >= 30; r -= 5) {
      if (direction == 1) {
          path[N++] = { 0.015f, -0.270f, 0.285f, DEG2RAD((float)r), 0.0f, DEG2RAD(50.0), 1000 };
          path[N++] = { 0.015f, -0.270f, 0.285f, DEG2RAD((float)r), 0.0f, DEG2RAD(-50.0), 1000 };
      } else {
          path[N++] = { 0.015f, -0.270f, 0.285f, DEG2RAD((float)r), 0.0f, DEG2RAD(-50.0), 1000 };
          path[N++] = { 0.015f, -0.270f, 0.285f, DEG2RAD((float)r), 0.0f, DEG2RAD(50.0), 1000 };
      }
      direction *= -1; // zmena smeru (cik-cak)
  }
  path[N++] = { 0.015f, -0.388f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(0.0), 1000 };

  // Po štarte systému necháme Control task chvíľu inicializovať servá
  vTaskDelay(pdMS_TO_TICKS(2000));

  while (1) {
    uint8_t cmd = TRAJ_CMD_NONE;
    
    // Zablokuj task a čakaj (task "spí", kým mu nepríde hlavička TRAJ_CMD_START)
    if (xQueueReceive(gUiToTrajectoryQueue, &cmd, portMAX_DELAY) == pdPASS) {
      
      if (cmd == TRAJ_CMD_START) {
        Serial.println("Trajectory START received");
        gTrajectoryRunning = 1;
        gTrajectoryStopRequested = 0;
        
        // Signalizujeme Control tasku, že preberáme moc
        taskENTER_CRITICAL();
        gTrajectoryTargetPose.active = 1;
        taskEXIT_CRITICAL();
        
        Serial.println("Trajectory running");

        for (int i = 0; i < N - 1; i++) {
          if (gTrajectoryStopRequested) {
              break; // Preruš celý cyklus vonkajších bodov pri stope
          }
          
          const Waypoint& a = path[i];
          const Waypoint& b = path[i + 1];

          uint32_t steps = b.duration_ms / Ts_ms;
          if (steps == 0) steps = 1;

          for (uint32_t s = 0; s <= steps; s++) {
            
            // Okamžité non-blocking načítanie STOP príkazu počas krokovania iterácie!
            uint8_t abortCmd = TRAJ_CMD_NONE;
            if (xQueueReceive(gUiToTrajectoryQueue, &abortCmd, 0) == pdPASS) {
               if (abortCmd == TRAJ_CMD_STOP) {
                   gTrajectoryStopRequested = 1;
                   Serial.println("Trajectory STOP received");
               }
            }

            if (gTrajectoryStopRequested) {
              break; // Okamžité prerušenie vnútorného cyklu cesty
            }

            float t = (float)s / (float)steps;

            float x = lerp(a.x, b.x, t);
            float y = lerp(a.y, b.y, t);
            float z = lerp(a.z, b.z, t);

            float roll  = lerp(a.roll,  b.roll,  t);
            float pitch = lerp(a.pitch, b.pitch, t);
            float yaw   = lerp(a.yaw,   b.yaw,   t);

            setTrajectoryPose(x, y, z, roll, pitch, yaw);

            vTaskDelay(pdMS_TO_TICKS(Ts_ms));
          }
        }
        RangingSensor::instance().enableLogging(false,false); // Stop logging after trajectory is done
        // Ukončenie / Návrat povelov do rúk UI Controlu
        taskENTER_CRITICAL();
        gTrajectoryTargetPose.valid = 0;
        gTrajectoryTargetPose.active = 0;
        taskEXIT_CRITICAL();

        gTrajectoryRunning = 0;
        Serial.println("Trajectory finished");
      }
    }
  }
}