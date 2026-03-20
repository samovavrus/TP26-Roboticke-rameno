#pragma once
#include "src\LiquidMenu\LiquidCrystal_I2C.h"

inline void drawXYZ(LiquidCrystal_I2C& lcd, float x, float y, float z) {
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("X:");
  lcd.print(x,1);

  lcd.setCursor(8,0);
  lcd.print("Y:");
  lcd.print(y,1);

  lcd.setCursor(0,1);
  lcd.print("Z:");
  lcd.print(z,1);
}

inline void drawRPY(LiquidCrystal_I2C& lcd, float r, float p, float y) {
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("R:");
  lcd.print(r,1);

  lcd.setCursor(8,0);
  lcd.print("P:");
  lcd.print(p,1);

  lcd.setCursor(0,1);
  lcd.print("Y:");
  lcd.print(y,1);
}