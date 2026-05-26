/**
 * @file ui_screens.h
 * @brief UI messages and LCD drawing helpers.
 */
#pragma once
#include <stdint.h>
#include <STM32FreeRTOS.h>
#include "src\\LiquidMenu\\LiquidCrystal_I2C.h"

/// @brief Control modes understood by the UI and control task.
enum UiControlMode : uint8_t {
	UI_CONTROL_MODE_POSE = 0,
	UI_CONTROL_MODE_JOINT = 1
};

/// @brief Control loop status codes.
enum UiControlStatus : uint8_t {
	UI_CONTROL_STATUS_OK = 0,
	UI_CONTROL_STATUS_IK_FAILED = 1,
	UI_CONTROL_STATUS_OUT_OF_LIMITS = 2
};

/// @brief UI -> Control command message payload.
typedef struct {
	uint8_t mode;
	float joint_rad[6];
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
} UiControlCommandMessage;

/// @brief Control -> UI state message payload.
typedef struct {
	uint8_t status;
	float joint_rad[6];
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
} UiControlStateMessage;

/// @brief Queue for UI -> Control commands.
extern QueueHandle_t gUiToControlQueue;
/// @brief Queue for Control -> UI state updates.
extern QueueHandle_t gControlToUiQueue;

/**
 * @brief Draw end-effector XYZ coordinates on the LCD.
 * @param lcd LCD instance.
 * @param x X coordinate (mm for UI display).
 * @param y Y coordinate (mm for UI display).
 * @param z Z coordinate (mm for UI display).
 * @param pair Label of the active axis pair.
 */
void drawXYZ(LiquidCrystal_I2C& lcd, float x, float y, float z, const char* pair);
/**
 * @brief Draw end-effector roll-pitch-yaw on the LCD.
 * @param lcd LCD instance.
 * @param r Roll angle (deg for UI display).
 * @param p Pitch angle (deg for UI display).
 * @param y Yaw angle (deg for UI display).
 * @param pair Label of the active axis pair.
 */
void drawRPY(LiquidCrystal_I2C& lcd, float r, float p, float y, const char* pair);
/**
 * @brief Draw joint angle pairs on the LCD.
 * @param lcd LCD instance.
 * @param t Joint angles array (deg for UI display).
 * @param mode Which joint pair to show (0..2).
 */
void drawJOINTS(LiquidCrystal_I2C& lcd, float* t, int mode);
/**
 * @brief FreeRTOS task entry for LCD UI and input handling.
 * @param pvParameters Unused task parameter.
 */
void TaskUI(void* pvParameters);
