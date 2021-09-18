#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <Windows.h>
#include <Xinput.h>

// XInput button values
static const WORD XINPUT_Buttons[] = {XINPUT_GAMEPAD_A,
                                      XINPUT_GAMEPAD_B,
                                      XINPUT_GAMEPAD_X,
                                      XINPUT_GAMEPAD_Y,
                                      XINPUT_GAMEPAD_DPAD_UP,
                                      XINPUT_GAMEPAD_DPAD_DOWN,
                                      XINPUT_GAMEPAD_DPAD_LEFT,
                                      XINPUT_GAMEPAD_DPAD_RIGHT,
                                      XINPUT_GAMEPAD_LEFT_SHOULDER,
                                      XINPUT_GAMEPAD_RIGHT_SHOULDER,
                                      XINPUT_GAMEPAD_LEFT_THUMB,
                                      XINPUT_GAMEPAD_RIGHT_THUMB,
                                      XINPUT_GAMEPAD_START,
                                      XINPUT_GAMEPAD_BACK};

// Used with the 'Gamepad' class to check for
// button presses. Each button has its own ID.
struct XInput_ButtonIDs
{
  XInput_ButtonIDs();

  // Action buttons
  int A;
  int B;
  int X;
  int Y;

  // Directional Pad (D-Pad)
  int DPad_Up;
  int DPad_Down;
  int DPad_Left;
  int DPad_Right;

  // Shoulder ('bumper') buttons
  int L_Shoulder;
  int R_Shoulder;

  // Thumbstick buttons (click in the thumbstick)
  int L_Thumbstick;
  int R_Thumbstick;

  int Start;  // START button
  int Back;   // BACK button
};

// Handles input from an Xbox 360 controller
class Gamepad
{
 public:
  Gamepad(int id);
  Gamepad(int id, double rDeadzone, double lDeadzone);

  void update();   // Update gamepad state
  void refresh();  // Prepare gamepad state for next cycle

  // Set gamepad vibration ('rumble')
  void setRumble(double left = 0.0, double right = 0.0);

  int getID();       // Return gamepad number
  bool connected();  // Check if gamepad is connected

  // Check if specified button is pressed
  bool getButtonPressed(int button);

  // Check if specified button is pressed - CURRENT frame only!
  bool getButtonDown(int button);

  // Check stick deadzones (true if inside, false if outside)
  bool lStick_InDeadzone();
  bool rStick_InDeadzone();

  double leftStick_X();   // Return X axis of left stick
  double leftStick_Y();   // Return Y axis of left stick
  double rightStick_X();  // Return X axis of right stick
  double rightStick_Y();  // Return Y axis of right stick

  double leftTrigger();   // Return value of left trigger
  double rightTrigger();  // Return value of right trigger

  XINPUT_STATE getState();  // Return gamepad state

 private:
  int pad_id;  // Gamepad number (1,2,3 or 4)

  static const int button_count = 14;

  bool prev_buttonStates[button_count];  // Button states from previous
                                         // update/frame
  bool buttonStates[button_count];  // Button states in current update/frame

  double LEFT_THUMB_DEADZONE;
  double RIGHT_THUMB_DEADZONE;

  XINPUT_STATE state;  // Current gamepad state
};

#endif

// Externally define 'XInput_ButtonIDs' struct as 'xButtons'
extern XInput_ButtonIDs xButtons;
