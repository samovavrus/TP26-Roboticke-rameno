#pragma once
#include <STM32FreeRTOS.h>
#include "src\\LiquidMenu\\LiquidCrystal_I2C.h"

typedef struct {
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
} DesiredPoseMessage;

extern QueueHandle_t gDesiredPoseQueue;

void drawXYZ(LiquidCrystal_I2C& lcd, float x, float y, float z, const char* pair);
void drawRPY(LiquidCrystal_I2C& lcd, float r, float p, float y, const char* pair);
void drawJOINTS(LiquidCrystal_I2C& lcd, float* t, const char* pair);
void TaskUI(void* pvParameters);
