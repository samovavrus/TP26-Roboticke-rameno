#pragma once

#include <STM32FreeRTOS.h>
#include <cstdint>

// Cielova poza generovana trajektoriou
typedef struct {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  uint8_t valid;    // 1 = data platne
  uint8_t active;   // 1 = trajektoria prave riadi robota
} TrajectoryTargetPose;

extern volatile TrajectoryTargetPose gTrajectoryTargetPose;

extern TaskHandle_t HandleTaskTrajectory;

void TaskTrajectory(void* pvParameters);

// Toto zavolas z UI
void Trajectory_StartScan(void);

// Toto mozes zavolat z UI, ak chces sken zastavit
void Trajectory_StopScan(void);

// Volitelne pre UI indikaciu
bool Trajectory_IsRunning(void);

// Definovaie správ
enum TrajectoryCommand : uint8_t {
    TRAJ_CMD_NONE = 0,
    TRAJ_CMD_START = 1,
    TRAJ_CMD_STOP = 2
};

// Deklarácia prístupu k fronte pre ostatné súbory
extern QueueHandle_t gUiToTrajectoryQueue;