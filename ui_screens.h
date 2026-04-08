#pragma once
#include "src\LiquidMenu\LiquidCrystal_I2C.h"

#pragma once
#include "src\LiquidMenu\LiquidCrystal_I2C.h"

inline void drawXYZ(LiquidCrystal_I2C& lcd, float x, float y, float z, const char* pair)
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

inline void drawRPY(LiquidCrystal_I2C& lcd, float r, float p, float y, const char* pair)
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