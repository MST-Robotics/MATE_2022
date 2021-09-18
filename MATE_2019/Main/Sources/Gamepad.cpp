#include "..\Headers\Gamepad.h"

// Link the 'XInput' library
// Note: For Visual Studio 2012 and above, XInput9_1_0 is the library required
// to make XInput work.
#pragma comment(lib, "XInput9_1_0.lib")

// Define the 'XInput_ButtonIDs' struct as 'xButtons'
XInput_ButtonIDs xButtons;

// Set buttton values in constructor
XInput_ButtonIDs::XInput_ButtonIDs()
{
  A = 0;
  B = 1;
  X = 2;
  Y = 3;

  DPad_Up = 4;
  DPad_Down = 5;
  DPad_Left = 6;
  DPad_Right = 7;

  L_Shoulder = 8;
  R_Shoulder = 9;

  L_Thumbstick = 10;
  R_Thumbstick = 11;

  Start = 12;
  Back = 13;
}

// Constructor - set pad ID
Gamepad::Gamepad(int id)
{
  pad_id = id - 1;
  state = getState();

  for (int i = 0; i < button_count; ++i)
  {
    prev_buttonStates[i] = false;
    buttonStates[i] = false;
  }

  LEFT_THUMB_DEADZONE = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
  RIGHT_THUMB_DEADZONE = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
}

// Constructor - set pad ID and joystick deadzones (percentage of max)
Gamepad::Gamepad(int id, double rDeadzone, double lDeadzone)
{
  pad_id = id - 1;
  state = getState();

  for (int i = 0; i < button_count; ++i)
  {
    prev_buttonStates[i] = false;
    buttonStates[i] = false;
  }

  // 65534.0 is maximum input
  LEFT_THUMB_DEADZONE = 65534.0 * lDeadzone;
  RIGHT_THUMB_DEADZONE = 65534.0 * rDeadzone;
}

// Update gamepad state
void Gamepad::update()
{
  // Get current gamepad state
  state = getState();

  for (int i = 0; i < button_count; ++i)
  {
    // Set button state on current frame
    buttonStates[i] =
        (state.Gamepad.wButtons & XINPUT_Buttons[i]) == XINPUT_Buttons[i];
  }
}

// Update gamepad state for next cycle
void Gamepad::refresh()
{
  memcpy(prev_buttonStates, buttonStates, sizeof(prev_buttonStates));
}

// Set gamepad vibration ('rumble')
void Gamepad::setRumble(double left, double right)
{
  // XInput vibration state
  XINPUT_VIBRATION rumble;

  // Zero memory on vibration state
  ZeroMemory(&rumble, sizeof(XINPUT_VIBRATION));

  // Calculate vibration intensity
  int left_motor = int(left * 65535.0);
  int right_motor = int(right * 65535.0);

  rumble.wLeftMotorSpeed = left_motor;
  rumble.wRightMotorSpeed = right_motor;

  // Apply vibration
  XInputSetState(pad_id, &rumble);
}

// Return gamepad number
int Gamepad::getID() { return pad_id; }

// Check if gamepad is connected
bool Gamepad::connected()
{
  // Zero memory
  ZeroMemory(&state, sizeof(XINPUT_STATE));

  // True = connected
  // False = not connected
  return XInputGetState(pad_id, &state) == ERROR_SUCCESS;
}

// Check if specified button is being pressed
bool Gamepad::getButtonPressed(int button)
{
  return state.Gamepad.wButtons & XINPUT_Buttons[button];
}

// Check if specified button was pressed - CURRENT frame only!
bool Gamepad::getButtonDown(int button)
{
  return !prev_buttonStates[button] && buttonStates[button];
}

// Check deadzone on left joystick
bool Gamepad::lStick_InDeadzone()
{
  return abs(state.Gamepad.sThumbLX) < LEFT_THUMB_DEADZONE &&
         abs(state.Gamepad.sThumbLY) < LEFT_THUMB_DEADZONE;
}

// Check deadzone on right joystick
bool Gamepad::rStick_InDeadzone()
{
  return abs(state.Gamepad.sThumbRX) < RIGHT_THUMB_DEADZONE &&
         abs(state.Gamepad.sThumbRY) < RIGHT_THUMB_DEADZONE;
}

// Return X axis of left joystick
double Gamepad::leftStick_X()
{
  if (!lStick_InDeadzone())
  {
    return state.Gamepad.sThumbLX / 32768.0;
  }
  else
  {
    return 0.0;
  }
}

// Return Y axis of left joystick
double Gamepad::leftStick_Y()
{
  if (!lStick_InDeadzone())
  {
    return state.Gamepad.sThumbLY / 32768.0;
  }
  else
  {
    return 0.0;
  }
}

// Return X axis of right joysick
double Gamepad::rightStick_X()
{
  if (!rStick_InDeadzone())
  {
    return state.Gamepad.sThumbRX / 32768.0;
  }
  else
  {
    return 0.0;
  }
}

// Return Y axis of right joystick
double Gamepad::rightStick_Y()
{
  if (!rStick_InDeadzone())
  {
    return state.Gamepad.sThumbRY / 32768.0;
  }
  else
  {
    return 0.0;
  }
}

// Return value of left trigger
double Gamepad::leftTrigger()
{
  BYTE axis = state.Gamepad.bLeftTrigger;

  if (axis > XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
  {
    return axis / 255.0;
  }
  else
  {
    return 0.0;
  }
}

// Return value of right trigger
double Gamepad::rightTrigger()
{
  BYTE axis = state.Gamepad.bRightTrigger;

  if (axis > XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
  {
    return axis / 255.0;
  }
  else
  {
    return 0.0;
  }
}

// Return gamepad state
XINPUT_STATE
Gamepad::getState()
{
  XINPUT_STATE new_state;

  // Zero memory on state
  ZeroMemory(&state, sizeof(XINPUT_STATE));

  // Get new state from XInput
  XInputGetState(pad_id, &new_state);

  return new_state;
}
