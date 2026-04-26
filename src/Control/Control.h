#pragma once

#include <STM32FreeRTOS.h>

extern TaskHandle_t HandleTaskControl;
extern QueueHandle_t gUiToControlQueue;
extern QueueHandle_t gControlToUiQueue;

void TaskControl(void* pvParameters);
