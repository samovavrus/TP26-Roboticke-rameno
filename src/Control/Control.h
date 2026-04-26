#pragma once

#include <STM32FreeRTOS.h>

#define CONTROL_START_THETA_0_RAD 0.0f
#define CONTROL_START_THETA_1_RAD (M_PI / 4.0f)
#define CONTROL_START_THETA_2_RAD (M_PI / 4.0f)
#define CONTROL_START_THETA_3_RAD 0.0f
#define CONTROL_START_THETA_4_RAD 0.0f
#define CONTROL_START_THETA_5_RAD 0.0f

#define CONTROL_IK_TOL_POS 1e-4f
#define CONTROL_IK_TOL_ORI 1e-3f
#define CONTROL_IK_LAMBDA 0.01f
#define CONTROL_IK_MAX_ITER 50

extern TaskHandle_t HandleTaskControl;
extern QueueHandle_t gUiToControlQueue;
extern QueueHandle_t gControlToUiQueue;

void TaskControl(void* pvParameters);
