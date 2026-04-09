#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "ui_screens.h"

void drawXYZ(LiquidCrystal_I2C& lcd, float x, float y, float z, const char* pair)
{
  lcd.setCursor(0,0);
  lcd.print("X:");
  lcd.print(x,1);
  lcd.print("   "); 

  lcd.setCursor(8,0);
  lcd.print("Y:");
  lcd.print(y,1);
  lcd.print("   ");

  lcd.setCursor(0,1);
  lcd.print("Z:");
  lcd.print(z,1);
  lcd.print("   ");

  lcd.setCursor(10,1);
  lcd.print(pair);
}

void drawRPY(LiquidCrystal_I2C& lcd, float r, float p, float y, const char* pair)
{
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("R:");
  lcd.print(r,1);
  lcd.print("   ");

  lcd.setCursor(8,0);
  lcd.print("P:");
  lcd.print(p,1);
  lcd.print("   ");

  lcd.setCursor(0,1);
  lcd.print("Y:");
  lcd.print(y,1);
  lcd.print("   ");

  lcd.setCursor(10,1);
  lcd.print(pair);
}

void TaskUI(void* pvParameters) {

  LiquidCrystal_I2C lcd(0x27, 16, 2);

  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();

  pinMode(2, INPUT_PULLUP); // Button A - change screen
  pinMode(3, INPUT_PULLUP); // Button B - change mode

  int screen = 0;
  int mode = 0;

  bool lastButtonA = HIGH;
  bool lastButtonB = HIGH;

  const char* pairXYZ[3] = {"XY","YZ","ZX"};
  const char* pairRPY[3] = {"RP","PY","YR"};

  // ---- persistent variables
  float x = 0;
  float y = 0;
  float z = 0;

  float roll  = 0;
  float pitch = 0;
  float yaw   = 0;

  while (1) {

    int joyX = analogRead(A0);
    int joyY = analogRead(A1);
    Serial.println(joyX);
    Serial.println(joyY);

    const int CENTER = 520;
    const int DEADZONE = 20;

    float dx = 0;
    float dy = 0;

    if (joyX > CENTER + DEADZONE)
      dx = (joyX - (CENTER + DEADZONE)) * 0.05;

    else if (joyX < CENTER - DEADZONE)
      dx = (joyX - (CENTER - DEADZONE)) * 0.05;

    if (joyY > CENTER + DEADZONE)
      dy = (joyY - (CENTER + DEADZONE)) * 0.05;

    else if (joyY < CENTER - DEADZONE)
      dy = (joyY - (CENTER - DEADZONE)) * 0.05;

    // ---- apply control based on mode
    switch(mode)
    {
      case 0: // XY / RP
        x += dx;
        y += dy;

        roll  += dx;
        pitch += dy;
      break;

      case 1: // YZ / PY
        y += dx;
        z += dy;

        pitch += dx;
        yaw   += dy;
      break;

      case 2: // ZX / YR
        z += dx;
        x += dy;

        yaw  += dx;
        roll += dy;
      break;
    }

    // ---- read buttons
    bool currentA = digitalRead(2);
    bool currentB = digitalRead(3);

    if (lastButtonA == HIGH && currentA == LOW) {
      screen = !screen;
      lcd.clear();
    }

    if (lastButtonB == HIGH && currentB == LOW) {
      mode = (mode + 1) % 3;
    }

    lastButtonA = currentA;
    lastButtonB = currentB;

    // ---- draw
    if (screen == 0) {
      drawXYZ(lcd, x, y, z, pairXYZ[mode]);
    } 
    else {
      drawRPY(lcd, roll, pitch, yaw, pairRPY[mode]);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
