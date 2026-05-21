#pragma once
#include <stdint.h>
#include <STM32FreeRTOS.h>
#include "src\\LiquidMenu\\LiquidCrystal_I2C.h"

enum UiControlMode : uint8_t {
	UI_CONTROL_MODE_POSE = 0,
	UI_CONTROL_MODE_JOINT = 1
};

enum UiControlStatus : uint8_t {
	UI_CONTROL_STATUS_OK = 0,
	UI_CONTROL_STATUS_IK_FAILED = 1,
	UI_CONTROL_STATUS_OUT_OF_LIMITS = 2
};

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

extern QueueHandle_t gUiToControlQueue;
extern QueueHandle_t gControlToUiQueue;

void drawXYZ(LiquidCrystal_I2C& lcd, float x, float y, float z, const char* pair);
void drawRPY(LiquidCrystal_I2C& lcd, float r, float p, float y, const char* pair);
void drawJOINTS(LiquidCrystal_I2C& lcd, float* t, int mode);
void TaskUI(void* pvParameters);
