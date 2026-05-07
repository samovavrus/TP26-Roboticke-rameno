#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "Trajectory.h"

TaskHandle_t HandleTaskTrajectory;

// Globálna cieľová póza pre Control task
volatile TrajectoryTargetPose gTrajectoryTargetPose = {0};

// Riadiace flagy pre spustenie/zastavenie trajektórie z UI
static volatile uint8_t gTrajectoryStartRequested = 0;
static volatile uint8_t gTrajectoryStopRequested  = 0;
static volatile uint8_t gTrajectoryRunning        = 0;

#define DEG2RAD(x) ((x) * 0.01745329252f)

struct Waypoint {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  uint32_t duration_ms;
};

static float lerp(float a, float b, float t) {
  return a + t * (b - a);
}

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

void Trajectory_StartScan(void) {
  gTrajectoryStartRequested = 1;
  gTrajectoryStopRequested = 0;
}

void Trajectory_StopScan(void) {
  gTrajectoryStopRequested = 1;
}

bool Trajectory_IsRunning(void) {
  return gTrajectoryRunning != 0;
}

void TaskTrajectory(void* pvParameters) {
  const uint32_t Ts_ms = 50;

const Waypoint path[] = {
    // Stred – neutrál
    {0.12f, 0.00f, 0.38f, DEG2RAD(0),  DEG2RAD(0),   DEG2RAD(0),   2000},

    // Ľavý kraj – dole
    {0.12f, 0.00f, 0.35f, DEG2RAD(0),  DEG2RAD(-10), DEG2RAD(-35), 2000},

    // Ľavý kraj – hore
    {0.12f, 0.00f, 0.40f, DEG2RAD(0),  DEG2RAD(20),  DEG2RAD(-35), 3000},

    // Medziľavý
    {0.12f, 0.00f, 0.40f, DEG2RAD(0),  DEG2RAD(20),  DEG2RAD(-20), 1200},
    {0.12f, 0.00f, 0.35f, DEG2RAD(0),  DEG2RAD(-10), DEG2RAD(-20), 3000},

    // Stred
    {0.12f, 0.00f, 0.40f, DEG2RAD(0),  DEG2RAD(20),  DEG2RAD(0),   1200},
    {0.12f, 0.00f, 0.35f, DEG2RAD(0),  DEG2RAD(-10), DEG2RAD(0),   3000},

    // Medzipravý
    {0.12f, 0.00f, 0.40f, DEG2RAD(0),  DEG2RAD(20),  DEG2RAD(20),  1200},
    {0.12f, 0.00f, 0.35f, DEG2RAD(0),  DEG2RAD(-10), DEG2RAD(20),  3000},

    // Pravý kraj
    {0.12f, 0.00f, 0.40f, DEG2RAD(0),  DEG2RAD(20),  DEG2RAD(35),  3000},

    // Návrat do stredu
    {0.12f, 0.00f, 0.38f, DEG2RAD(0),  DEG2RAD(0),   DEG2RAD(0),   2000}
};

  const int N = sizeof(path) / sizeof(path[0]);

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