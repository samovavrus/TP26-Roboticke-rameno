#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "src/Keypad/Keypad.h"

#include "ui_screens.h"

#define USE_KEYPAD  1   // 0 = dev buttons (pins 2,3), 1 = keypad buttons

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

void drawJOINTS(LiquidCrystal_I2C& lcd, float* t, int mode)
{
  int i1 = mode * 2;
  int i2 = mode * 2 + 1;

  lcd.setCursor(0,0);
  lcd.print("T");
  lcd.print(i1+1);
  lcd.print(":");
  lcd.print(t[i1],1);
  lcd.print("   ");

  lcd.setCursor(8,0);
  lcd.print("T");
  lcd.print(i2+1);
  lcd.print(":");
  lcd.print(t[i2],1);
  lcd.print("   ");

  lcd.setCursor(0,1);
  lcd.print("PAIR:");
  lcd.print(i1+1);
  lcd.print(i2+1);
}

void TaskUI(void* pvParameters) {

  LiquidCrystal_I2C lcd(0x27, 16, 2);

  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();

  pinMode(2, INPUT_PULLUP); // Button A - change screen
  pinMode(3, INPUT_PULLUP); // Button B - change mode

  #if USE_KEYPAD
    const char keys[4][4] = {
      { '1', '2', '3', 'A' },
      { '4', '5', '6', 'B' },
      { '7', '8', '9', 'C' },
      { '.', '0', '-', 'D' }
    };

    byte rowPins[4] = { 4, 5, 6, 7 };
    byte colPins[4] = { 8, 9, 10, 11 };

    Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);
  #endif

  int screen = 0; // 0=XYZ, 1=RPY, 2=JOINTS
  int mode = 0;

  static bool lastA = false;
  static bool lastB = false; 

  const char* pairXYZ[3] = {"XY","YZ","ZX"};
  const char* pairRPY[3] = {"RP","PY","YR"};

  // Test variables
  float x = 15.0f;
  float y = -388.2f;
  float z = 155.7f;

  float roll  = 90.0f;
  float pitch = 0.0f;
  float yaw   = 0.0f;

  float servoAngles[6] = {0,0,0,0,0,0};

  const float joint_min[6] = {-180,-90,-90,-180,-120,-180};
  const float joint_max[6] = { 180, 90, 90, 180, 120, 180};

  while (1) {
    // ---- JOYSTICK ----
    int joyX = analogRead(A0);
    int joyY = analogRead(A1);

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

    // ---- BUTTONS ----
    bool btnA = false;
    bool btnB = false;

    #if USE_KEYPAD
      char key = keypad.getKey();
      if (key == 'A') btnA = true;
      if (key == 'B') btnB = true;
    #else
      btnA = (digitalRead(2) == LOW);
      btnB = (digitalRead(3) == LOW);
    #endif

    if (!lastA && btnA) {
      screen = (screen + 1) % 3;
      lcd.clear();
    }

    if (!lastB && btnB) {
      mode = (mode + 1) % 3;
    }

    lastA = btnA;
    lastB = btnB;

    // ---- SCREEN ----
  if (screen == 0) {
    // XYZ
    switch(mode)
    {
      case 0: x += dx; y += dy; break;
      case 1: y += dx; z += dy; break;
      case 2: z += dx; x += dy; break;
    }
    drawXYZ(lcd, x, y, z, pairXYZ[mode]);
  }
  else if (screen == 1) {
    // RPY
    switch(mode)
    {
      case 0: roll += dx; pitch += dy; break;
      case 1: pitch += dx; yaw += dy; break;
      case 2: yaw += dx; roll += dy; break;
    }
    drawRPY(lcd, roll, pitch, yaw, pairRPY[mode]);
  }
  else {
    // JOINTS
    switch(mode)
    {
      case 0: servoAngles[0] += dx; servoAngles[1] += dy; break;
      case 1: servoAngles[2] += dx; servoAngles[3] += dy; break;
      case 2: servoAngles[4] += dx; servoAngles[5] += dy; break;
    }

    for(int i=0;i<6;i++)
      servoAngles[i] = constrain(servoAngles[i], joint_min[i], joint_max[i]);

    drawJOINTS(lcd, servoAngles, mode);
  }

    if (gDesiredPoseQueue != NULL) {
      DesiredPoseMessage msg = {
        x / 1000.0f,
        y / 1000.0f,
        z / 1000.0f,
        roll * DEG_TO_RAD,
        pitch * DEG_TO_RAD,
        yaw * DEG_TO_RAD
      };
      (void)xQueueSend(gDesiredPoseQueue, &msg, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
