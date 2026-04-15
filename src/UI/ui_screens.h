#pragma once
#include "src\\LiquidMenu\\LiquidCrystal_I2C.h"

void drawXYZ(LiquidCrystal_I2C& lcd, float x, float y, float z, const char* pair);
void drawRPY(LiquidCrystal_I2C& lcd, float r, float p, float y, const char* pair);
void TaskUI(void* pvParameters);
