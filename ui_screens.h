#pragma once

#include "src\LiquidMenu\LiquidCrystal_I2C.h"

/**
 * @file ui_screens.h
 * @brief User interface display functions for LCD output
 * 
 * Provides inline utility functions for rendering robot arm state information
 * on a 16×2 character LCD display via I2C connection (LiquidCrystal_I2C).
 * 
 * Functions display:
 * - End-effector position (X, Y, Z Cartesian coordinates)
 * - End-effector orientation (Roll-Pitch-Yaw Euler angles)
 * 
 * All values are formatted to 1 decimal place for readability on small displays.
 */

/**
 * @brief Displays end-effector position (X, Y, Z) on LCD screen
 * @param lcd Reference to LiquidCrystal_I2C display object
 * @param x End-effector X position in meters
 * @param y End-effector Y position in meters
 * @param z End-effector Z position in meters
 * 
 * Display layout (16×2 character LCD):
 * ```
 * Row 0: "X:xxx.x  Y:yyy.y"
 * Row 1: "Z:zzz.z"
 * ```
 * 
 * All coordinates are displayed with 1 decimal place precision.
 * Screen is cleared before rendering.
 * 
 * @note Used during manual control and end-effector monitoring modes.
 *       Typical refresh rate: 100-500 ms (depends on control loop frequency).
 */
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

/**
 * @brief Displays end-effector orientation (Roll-Pitch-Yaw) on LCD screen
 * @param lcd Reference to LiquidCrystal_I2C display object
 * @param r Roll angle in radians (rotation about X-axis)
 * @param p Pitch angle in radians (rotation about Y-axis)
 * @param y Yaw angle in radians (rotation about Z-axis)
 * 
 * Display layout (16×2 character LCD):
 * ```
 * Row 0: "R:rrr.r  P:ppp.p"
 * Row 1: "Y:yyy.y"
 * ```
 * 
 * All angles are displayed with 1 decimal place precision.
 * Screen is cleared before rendering.
 * 
 * Rotation convention (right-hand rule):
 * - Roll (R):  Rotation about X-axis (shoulder tilt)
 * - Pitch (P): Rotation about Y-axis (elbow elevation)
 * - Yaw (Y):   Rotation about Z-axis (wrist rotation)
 * 
 * @note Used during manual control and end-effector orientation monitoring modes.
 *       Angles are typically stored internally in radians but displayed here
 *       as-is for direct verification against joint sensor values.
 *       Consider converting to degrees for user-friendlier display if needed.
 */
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