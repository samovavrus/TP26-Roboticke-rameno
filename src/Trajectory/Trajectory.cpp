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
    // x       y       z       roll           pitch            yaw             time

    // Bezpečná stredová pozícia pred skenovaním
    {0.12f,  0.00f,  0.38f,  DEG2RAD(0),    DEG2RAD(0),      DEG2RAD(0),     2000},

    // Ľavý kraj, pohľad nižšie
    {0.12f,  0.00f,  0.34f,  DEG2RAD(0),    DEG2RAD(-15),    DEG2RAD(-45),   2000},

    // Ľavý kraj, plynulo hore
    {0.12f,  0.00f,  0.42f,  DEG2RAD(0),    DEG2RAD(30),     DEG2RAD(-45),   3500},

    // Pootocenie okolo Z
    {0.12f,  0.00f,  0.42f,  DEG2RAD(0),    DEG2RAD(30),     DEG2RAD(-30),   1200},

    // Skenovanie smerom dole
    {0.12f,  0.00f,  0.34f,  DEG2RAD(0),    DEG2RAD(-15),    DEG2RAD(-30),   3500},

    // Pootocenie okolo Z
    {0.12f,  0.00f,  0.34f,  DEG2RAD(0),    DEG2RAD(-15),    DEG2RAD(-15),   1200},

    // Skenovanie smerom hore
    {0.12f,  0.00f,  0.42f,  DEG2RAD(0),    DEG2RAD(30),     DEG2RAD(-15),   3500},

    // Pootocenie do stredu
    {0.12f,  0.00f,  0.42f,  DEG2RAD(0),    DEG2RAD(30),     DEG2RAD(0),     1200},

    // Skenovanie smerom dole
    {0.12f,  0.00f,  0.34f,  DEG2RAD(0),    DEG2RAD(-15),    DEG2RAD(0),     3500},

    // Pootocenie doprava
    {0.12f,  0.00f,  0.34f,  DEG2RAD(0),    DEG2RAD(-15),    DEG2RAD(15),    1200},

    // Skenovanie smerom hore
    {0.12f,  0.00f,  0.42f,  DEG2RAD(0),    DEG2RAD(30),     DEG2RAD(15),    3500},

    // Pootocenie doprava
    {0.12f,  0.00f,  0.42f,  DEG2RAD(0),    DEG2RAD(30),     DEG2RAD(30),    1200},

    // Skenovanie smerom dole
    {0.12f,  0.00f,  0.34f,  DEG2RAD(0),    DEG2RAD(-15),    DEG2RAD(30),    3500},

    // Pootocenie na pravý kraj
    {0.12f,  0.00f,  0.34f,  DEG2RAD(0),    DEG2RAD(-15),    DEG2RAD(45),    1200},

    // Pravý kraj, smerom hore
    {0.12f,  0.00f,  0.42f,  DEG2RAD(0),    DEG2RAD(30),     DEG2RAD(45),    3500},

    // Návrat do stredu
    {0.12f,  0.00f,  0.38f,  DEG2RAD(0),    DEG2RAD(0),      DEG2RAD(0),     2500}
  };

  const int N = sizeof(path) / sizeof(path[0]);

  // Po štarte systému necháme Control task chvíľu inicializovať servá
  vTaskDelay(pdMS_TO_TICKS(2000));

  while (1) {
    // Čakáme na požiadavku z UI
    if (!gTrajectoryStartRequested) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    gTrajectoryStartRequested = 0;
    gTrajectoryStopRequested = 0;
    gTrajectoryRunning = 1;

    Serial.println("Trajectory scan started");

    for (int i = 0; i < N - 1; i++) {
      if (gTrajectoryStopRequested) {
        break;
      }

      const Waypoint& a = path[i];
      const Waypoint& b = path[i + 1];

      uint32_t steps = b.duration_ms / Ts_ms;
      if (steps == 0) {
        steps = 1;
      }

      for (uint32_t s = 0; s <= steps; s++) {
        if (gTrajectoryStopRequested) {
          break;
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

    // Po skončení trajektórie vypneme trajectory override
    taskENTER_CRITICAL();
    gTrajectoryTargetPose.valid = 0;
    taskEXIT_CRITICAL();

    gTrajectoryRunning = 0;
    gTrajectoryStopRequested = 0;

    Serial.println("Trajectory scan finished");
  }
}