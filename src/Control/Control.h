/**
 * @file Control.h
 * @brief FreeRTOS control task interface and IK configuration.
 */
#pragma once

#include <STM32FreeRTOS.h>

/** @brief Startup joint angle 0 in radians. */
#define CONTROL_START_THETA_0_RAD 0.0f
/** @brief Startup joint angle 1 in radians. */
#define CONTROL_START_THETA_1_RAD (M_PI / 4.0f)
/** @brief Startup joint angle 2 in radians. */
#define CONTROL_START_THETA_2_RAD (M_PI / 4.0f)
/** @brief Startup joint angle 3 in radians. */
#define CONTROL_START_THETA_3_RAD 0.0f
/** @brief Startup joint angle 4 in radians. */
#define CONTROL_START_THETA_4_RAD 0.0f
/** @brief Startup joint angle 5 in radians. */
#define CONTROL_START_THETA_5_RAD 0.0f

/** @brief IK position tolerance in meters. */
#define CONTROL_IK_TOL_POS 1e-4f
/** @brief IK orientation tolerance in radians. */
#define CONTROL_IK_TOL_ORI 1e-3f
/** @brief IK damping factor for Levenberg-Marquardt. */
#define CONTROL_IK_LAMBDA 0.01f
/** @brief IK maximum iteration count. */
#define CONTROL_IK_MAX_ITER 50

/// @brief FreeRTOS task handle for the main control task.
extern TaskHandle_t HandleTaskControl;
/// @brief Queue for UI -> Control commands.
extern QueueHandle_t gUiToControlQueue;
/// @brief Queue for Control -> UI state updates.
extern QueueHandle_t gControlToUiQueue;

/**
 * @brief FreeRTOS task entry point for robot control.
 * @param pvParameters Unused task parameter.
 */
void TaskControl(void* pvParameters);
