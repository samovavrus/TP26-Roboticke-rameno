#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "src/Keypad/Keypad.h"
#include "src/Trajectory/Trajectory.h" 

#include "ui_screens.h"

#define USE_KEYPAD  1   // 0 = dev buttons (pins 2,3), 1 = keypad buttons

static void applyControlStateToUi(const UiControlStateMessage& state,
                                  float& x,
                                  float& y,
                                  float& z,
                                  float& roll,
                                  float& pitch,
                                  float& yaw,
                                  float* servoAngles)
{
  x = state.x * 1000.0f;
  y = state.y * 1000.0f;
  z = state.z * 1000.0f;
  roll = state.roll * RAD_TO_DEG;
  pitch = state.pitch * RAD_TO_DEG;
  yaw = state.yaw * RAD_TO_DEG;

  for (int i = 0; i < 6; ++i) {
    servoAngles[i] = state.joint_rad[i] * RAD_TO_DEG;
  }
}

float applyDeadzone(int value, int center, int deadzone, float scale)
{
  float delta = (float)value - center;

  if (fabs(delta) < deadzone)
    return 0.0f;

  float sign = (delta > 0) ? 1.0f : -1.0f;
  float magnitude = fabs(delta) - deadzone;

  float maxRange = 512.0f - deadzone;
  float normalized = magnitude / maxRange;

  return sign * normalized * scale;
}

static void showFatalControlError(LiquidCrystal_I2C& lcd, uint8_t status)
{
  const bool isIkError = (status == UI_CONTROL_STATUS_IK_FAILED);

  lcd.clear();
  while (1) {
    lcd.setCursor(0, 0);
    lcd.print("CONTROL ERROR   ");

    lcd.setCursor(0, 1);
    if (isIkError) {
      lcd.print("IK FAILED       ");
    } else {
      lcd.print("OUT OF LIMITS   ");
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

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

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float roll  = 0.0f;
  float pitch = 0.0f;
  float yaw   = 0.0f;

  float servoAngles[6] = {0,0,0,0,0,0};
  UiControlStateMessage latestState = {};
  bool hasState = false;

  if (gControlToUiQueue != NULL) {
    UiControlStateMessage startupState;
    // Block UI startup until TaskControl publishes the first state snapshot.
    while (xQueueReceive(gControlToUiQueue, &startupState, pdMS_TO_TICKS(100)) != pdPASS) {
    }

    latestState = startupState;
    hasState = true;
    if (startupState.status == UI_CONTROL_STATUS_IK_FAILED || startupState.status == UI_CONTROL_STATUS_OUT_OF_LIMITS) {
      showFatalControlError(lcd, startupState.status);
    }
    applyControlStateToUi(startupState, x, y, z, roll, pitch, yaw, servoAngles);
  }

  const float joint_min[6] = {-180,-90,-90,-180,-120,-180};
  const float joint_max[6] = { 180, 90, 90, 180, 120, 180};

  while (1) {
    if (gControlToUiQueue != NULL) {
      UiControlStateMessage rxState;
      bool updated = false;
      while (xQueueReceive(gControlToUiQueue, &rxState, 0) == pdPASS) {
        latestState = rxState;
        updated = true;
      }

      if (updated) {
        hasState = true;
        if (latestState.status == UI_CONTROL_STATUS_IK_FAILED || latestState.status == UI_CONTROL_STATUS_OUT_OF_LIMITS) {
          showFatalControlError(lcd, latestState.status);
        }
        applyControlStateToUi(latestState, x, y, z, roll, pitch, yaw, servoAngles);
      }
    }

    // ---- JOYSTICK ----
    int joyX = analogRead(A0);
    int joyY = analogRead(A1);

    const int CENTER = 520;
    const int DEADZONE = 20;

    float dx = applyDeadzone(joyX, CENTER, DEADZONE, 0.05f);
    float dy = applyDeadzone(joyY, CENTER, DEADZONE, 0.05f);

    // ---- BUTTONS ----
    bool btnA = false;
    bool btnB = false;
    bool startTrajectory = false;
    bool stopTrajectory = false; 

    #if USE_KEYPAD
      char key = keypad.getKey();
      if (key == 'A') btnA = true;
      if (key == 'B') btnB = true;
      if (key == 'C') startTrajectory = true; 
      if (key == 'D') stopTrajectory  = true;
    #else
      btnA = (digitalRead(2) == LOW);
      btnB = (digitalRead(3) == LOW);
      startTrajectory = (digitalRead(12) == LOW);
      stopTrajectory  = (digitalRead(13) == LOW);
    #endif

    if (!lastA && btnA) {
      const int previousScreen = screen;
      screen = (screen + 1) % 3;
      if (hasState && ((previousScreen == 2) != (screen == 2))) {
        // On JOINT <-> POSE mode switches, refresh from last confirmed control state.
        applyControlStateToUi(latestState, x, y, z, roll, pitch, yaw, servoAngles);
      }
      mode = 0;
      lcd.clear();
    }

    if (!lastB && btnB) {
      mode = (mode + 1) % 3;
    }

    lastA = btnA;
    lastB = btnB;

    // Spustenie trajektórie - Iba ak nebeží
    if (startTrajectory && !Trajectory_IsRunning() && (gUiToTrajectoryQueue != NULL)) {
      uint8_t cmd = TRAJ_CMD_START;
      xQueueOverwrite(gUiToTrajectoryQueue, &cmd);
    }

    // Zastavenie trajektórie - Okamžitý stop
    if (stopTrajectory && Trajectory_IsRunning() && (gUiToTrajectoryQueue != NULL)) {
      uint8_t cmd = TRAJ_CMD_STOP;
      xQueueOverwrite(gUiToTrajectoryQueue, &cmd);
    }

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

    if (gUiToControlQueue != NULL) {
      UiControlCommandMessage msg = {};
      msg.mode = (screen == 2) ? UI_CONTROL_MODE_JOINT : UI_CONTROL_MODE_POSE;
      for (int i = 0; i < 6; ++i) {
        msg.joint_rad[i] = servoAngles[i] * DEG_TO_RAD;
      }

      msg.x = x / 1000.0f;
      msg.y = y / 1000.0f;
      msg.z = z / 1000.0f;
      msg.roll = roll * DEG_TO_RAD;
      msg.pitch = pitch * DEG_TO_RAD;
      msg.yaw = yaw * DEG_TO_RAD;

      (void)xQueueOverwrite(gUiToControlQueue, &msg);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
