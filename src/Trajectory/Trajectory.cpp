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
    // Vrstva 0 -> od 0 do 90
    { 0.015f, -0.388f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.049f, -0.385f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(5.0), 500 },
    { 0.082f, -0.380f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(10.0), 500 },
    { 0.115f, -0.371f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(15.0), 500 },
    { 0.147f, -0.360f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(20.0), 500 },
    { 0.178f, -0.346f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(25.0), 500 },
    { 0.207f, -0.329f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(30.0), 500 },
    { 0.235f, -0.309f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(35.0), 500 },
    { 0.261f, -0.288f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(40.0), 500 },
    { 0.285f, -0.264f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(45.0), 500 },
    { 0.307f, -0.238f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(50.0), 500 },
    { 0.327f, -0.210f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(55.0), 500 },
    { 0.344f, -0.181f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(60.0), 500 },
    { 0.358f, -0.150f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(65.0), 500 },
    { 0.370f, -0.119f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(70.0), 500 },
    { 0.379f, -0.086f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(75.0), 500 },
    { 0.385f, -0.053f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(80.0), 500 },
    { 0.388f, -0.019f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(85.0), 500 },
    { 0.388f, 0.015f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    // Zdvihnutie 0 -> 1
    { 0.35f, 0.015f, 0.183f, DEG2RAD(85.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    { 0.35f, 0.015f, 0.210f, DEG2RAD(80.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    { 0.35f, 0.015f, 0.237f, DEG2RAD(75.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    { 0.35f, 0.015f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    // Vrstva 1 -> od 90 do 0
    { 0.373f, -0.018f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(85.0), 500 },
    { 0.371f, -0.050f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(80.0), 500 },
    { 0.365f, -0.082f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(75.0), 500 },
    { 0.356f, -0.114f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(70.0), 500 },
    { 0.345f, -0.144f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(65.0), 500 },
    { 0.331f, -0.174f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(60.0), 500 },
    { 0.315f, -0.202f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(55.0), 500 },
    { 0.296f, -0.229f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(50.0), 500 },
    { 0.275f, -0.254f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(45.0), 500 },
    { 0.252f, -0.277f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(40.0), 500 },
    { 0.227f, -0.297f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(35.0), 500 },
    { 0.200f, -0.316f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(30.0), 500 },
    { 0.171f, -0.332f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(25.0), 500 },
    { 0.142f, -0.346f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(20.0), 500 },
    { 0.111f, -0.357f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(15.0), 500 },
    { 0.080f, -0.365f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(10.0), 500 },
    { 0.048f, -0.371f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(5.0), 500 },
    { 0.015f, -0.374f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    // Zdvihnutie 1 -> 2
    { 0.015f, -0.364f, 0.290f, DEG2RAD(65.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.352f, 0.314f, DEG2RAD(60.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.339f, 0.338f, DEG2RAD(55.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.323f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    // Vrstva 2 -> od 0 do 90
    { 0.043f, -0.320f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(5.0), 500 },
    { 0.071f, -0.315f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(10.0), 500 },
    { 0.098f, -0.308f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(15.0), 500 },
    { 0.125f, -0.298f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(20.0), 500 },
    { 0.150f, -0.286f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(25.0), 500 },
    { 0.174f, -0.272f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(30.0), 500 },
    { 0.197f, -0.256f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(35.0), 500 },
    { 0.219f, -0.238f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(40.0), 500 },
    { 0.239f, -0.218f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(45.0), 500 },
    { 0.257f, -0.196f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(50.0), 500 },
    { 0.273f, -0.173f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(55.0), 500 },
    { 0.287f, -0.148f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(60.0), 500 },
    { 0.299f, -0.123f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(65.0), 500 },
    { 0.308f, -0.096f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(70.0), 500 },
    { 0.316f, -0.069f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(75.0), 500 },
    { 0.321f, -0.041f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(80.0), 500 },
    { 0.323f, -0.013f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(85.0), 500 },
    { 0.323f, 0.015f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    // Zdvihnutie 2 -> 3
    { 0.305f, 0.015f, 0.381f, DEG2RAD(45.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    { 0.286f, 0.015f, 0.401f, DEG2RAD(40.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    { 0.265f, 0.015f, 0.418f, DEG2RAD(35.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    { 0.242f, 0.015f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(90.0), 500 },
    // Vrstva 3 -> od 90 do 0
    { 0.242f, -0.006f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(85.0), 500 },
    { 0.241f, -0.027f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(80.0), 500 },
    { 0.238f, -0.048f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(75.0), 500 },
    { 0.233f, -0.069f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(70.0), 500 },
    { 0.226f, -0.089f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(65.0), 500 },
    { 0.217f, -0.108f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(60.0), 500 },
    { 0.207f, -0.127f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(55.0), 500 },
    { 0.195f, -0.144f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(50.0), 500 },
    { 0.182f, -0.161f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(45.0), 500 },
    { 0.167f, -0.176f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(40.0), 500 },
    { 0.151f, -0.190f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(35.0), 500 },
    { 0.134f, -0.202f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(30.0), 500 },
    { 0.116f, -0.213f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(25.0), 500 },
    { 0.097f, -0.222f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(20.0), 500 },
    { 0.077f, -0.230f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(15.0), 500 },
    { 0.057f, -0.236f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(10.0), 500 },
    { 0.036f, -0.240f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(5.0), 500 },
    { 0.015f, -0.242f, 0.434f, DEG2RAD(30.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    // Navrat domov (klesanie)
    { 0.015f, -0.265f, 0.418f, DEG2RAD(35.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.286f, 0.401f, DEG2RAD(40.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.305f, 0.381f, DEG2RAD(45.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.323f, 0.361f, DEG2RAD(50.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.339f, 0.338f, DEG2RAD(55.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.352f, 0.314f, DEG2RAD(60.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.364f, 0.290f, DEG2RAD(65.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.374f, 0.264f, DEG2RAD(70.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.381f, 0.237f, DEG2RAD(75.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.386f, 0.210f, DEG2RAD(80.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.388f, 0.183f, DEG2RAD(85.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
    { 0.015f, -0.388f, 0.156f, DEG2RAD(90.0), DEG2RAD(-0.0), DEG2RAD(0.0), 500 },
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