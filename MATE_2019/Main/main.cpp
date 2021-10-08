#include <iostream>

#include ".\Headers\Gamepad.h"
#include ".\Headers\PID.h"
#include ".\Headers\SerialPort.h"
#include ".\Headers\Utils.h"

using namespace std;

bool disabled = false;
bool waterLeak = false;
bool manual = true; // Added manual variable to distinguish between when to use controller vs camera data

char output[MAX_DATA_LENGTH]; // This is currently 255
int pitch = 0;  // Heading up and down
int roll = 0;   // Angle side to side relative to ground
int yawOffset = 0;

double pitchSetpoint = 0.0;
double rollSetpoint = 0.0;

int sleepTime = 90; // Added a new variable to easily change the command sleep time

// Change the name of the port with the port name of your computer
// Must remember that the backslashes are essential so do not remove them
const char* port = "\\\\.\\COM3";
SerialPort arduino(port, 115200);
Gamepad gamepad1 = Gamepad(1);
Gamepad gamepad2 = Gamepad(2);
Gamepad activeGamepad; // Added placeholder variable for either of the gamepads
bool gamepadToggle = true; // Added gamepad toggle for easy switching, true is gamepad1, false is gamepad2

// Sends the command string to the arduino and echoes back the returned output
void transferData(string data)
{
  // Convert data string to character array
  char* charArray = new char[data.size()];
  copy(data.begin(), data.end(), charArray);
  // Send command array to arduino
  arduino.writeSerialPort(charArray, data.size() - 1);
  delete[] charArray;

  // Wait for arduino to process command
  Sleep(sleepTime); //This value will need to be adjusted as we test the ROV (how long each command typically takes)

  // Expects to receive the command string echoed back, unless there is a water leak, then it will just recieve 'W'
  arduino.readSerialPort(output, MAX_DATA_LENGTH);

  // The loop will output each of the characters in the output array
  cout << " Received: ";
  for (char c : output)
  {
    cout << c;
    if (c == 'W') // Changed this to 'W' instead of '1', because a 1 will likely be sent in the command string
    {
      waterLeak = true;
    }
  }
  cout << endl << endl;

  // If a 'W' was returned by the arduino, then waterLeak would have been changed to true
  if (waterLeak) 
  {
    for (int i = 0; i < 50; ++i)
    {
      cout << "WATER "; // Spams WATER in the console
    }
  }
}

// Processes the gamepad inputs or camera data and converts it into a command string
void teleop(int controlMethod) // Added integer argument for control method (0 is camera, 1 is gamepad), rather than passing arbitrary controller values
{

  // : is verification character for arduino
  string data = ":";

  /* Not sure what these do
  PID pitchPID(0.007, 0.0, 0.0);
  pitchPID.setContinuous(false);
  pitchPID.setOutputLimits(-1.0, 1.0);
  pitchPID.setSetpoint(pitchSetpoint);

  PID rollPID(0.007, 0.0, 0.0);
  rollPID.setContinuous(false);
  rollPID.setOutputLimits(-1.0, 1.0);
  rollPID.setSetpoint(rollSetpoint);
  */
  
  // Skip all calculations if we are simply disabling the ROV
  if (disabled) 
  {
    data = ":000000000000000000000\n"; // Sets to a command string that will turn everything off
  }
  // Manual control section
  // THIS SECTION MUST BE REWORKED IN ORDER TO PRODUCE VALUES FROM 00-99 IN ORDER TO ALIGN WITH OTHER SECTIONS
  else if (controlMethod == 1) {

    // Let driver adjust angle of robot if necessary
    if (gamepad1.getButtonPressed(xButtons.A))
    {
      pitchSetpoint += -0.01;
    }
    else if (gamepad1.getButtonPressed(xButtons.Y))
    {
      pitchSetpoint += 0.01;
    }

    if (gamepad1.getButtonPressed(xButtons.B))
    {
      rollSetpoint += -0.01;
    }
    else if (gamepad1.getButtonPressed(xButtons.X))
    {
      rollSetpoint += 0.01;
    }

    // Will not reach full power diagonally because of controller input (depending
    // on controller)
    const double rad45 = 45.0 * 3.14159 / 180.0;

    // heading adjusts where front is
    double heading = -rad45;

    // FR AND FL ARE SWAPPED
    // BR IS INVERTED?
    // WORKING VALUES 2/13/2020 W/ EXTENSION CORDS
    double UL = -(gamepad1.rightTrigger() - gamepad1.leftTrigger()); // Left propulsion
    double UR = -(gamepad1.rightTrigger() - gamepad1.leftTrigger()); // Right propulsion
    double UB = -(gamepad1.rightTrigger() - gamepad1.leftTrigger()) * 0.4; // Upwards propulsion

    double FL = (-STR * cos(heading) + FWD * cos(heading) - RCW);    // Front left motor
    double BL = (-STR * sin(heading) + FWD * cos(heading) - RCW);  // Back left motor
    double FR = (-STR * sin(heading) - FWD * sin(heading) + RCW);   // Front right motor
    double BR = (STR * cos(heading) + FWD * sin(heading) - RCW);    // Back right motor
 
    double* vals[] = {&UL, &UR, &UB, &FL, &BL, &FR, &BR}; // Ordered according to declaration/command string
  
    double max = 1.0;

    // Normalize the horizontal motor powers if calculation goes above 100%
    for (int i = 0; i < 4; ++i)
    {
      if (abs(*vals[i]) > max)
      {
        max = abs(*vals[i]);
      }
    }

    for (int i = 0; i < 4; ++i)
    {
      *vals[i] /= max;
    }

    // Normalize the vertical motor powers if calculation goes above 100%
    for (int i = 4; i < 7; ++i)
    {
      if (abs(*vals[i]) > max)
      {
        max = abs(*vals[i]);
      }
    }

    for (int i = 4; i < 7; ++i)
    {
      *vals[i] /= max;
    }
  
    // Don't send command if it is below a certain threshold
    // Or the robot is disabled
    for (double* num : vals)
    {
      if (abs(*num) < 0.1 || disabled)
      {
        *num = 0.0;
      }
    }
 
    // Convert the values to something the motors can read
    for (double* num : vals)
    {
      *num = Utils::convertRange(-1.0, 1.0, 1100.0, 1900.0, *num);
      data.append(to_string((int)*num) + ";");
    }

    double shoulder = 0.0;
    double wristTilt = 0.0;
    double wristTwist = 0.0;

    shoulder += gamepad2.leftStick_Y() * 0.05;
    wristTilt += gamepad2.rightStick_Y() * 0.05;
    wristTwist += gamepad2.rightStick_X() * 0.5;
    double* armVals[] = {&shoulder, &wristTilt, &wristTwist};

    for (double* num : armVals)
    {
      *num = Utils::convertRange(-1.0, 1.0, 1100.0, 1900.0, *num);
      data.append(to_string((int)*num) + ";");
    }

    // Claw command + end of command string character
    data.append(to_string((int)gamepad1.getButtonPressed(xButtons.A)) + "\n");

  }
  //Autonomous camera control section
  else 
  {
    data = ":000000000000000000000\n"; // Sets to a command string that will turn everything off
    // The autonomous calculations and functionality will go here
    // This means that the program will need to be fed the camera (x, y, z) here, and that these coordinates + angles are used in order to set motor values
  }

  // Send the command string to the transferData function
  cout << " Sending: " << data;
  transferData(data);

}

// Main function processes inputs into a command string that controls the arduino
int main()
{

  // Loops until the arduino is connected
  while (!arduino.isConnected())
  {
    cout << " Error in Arduino port name" << endl << endl;
  }
  cout << " Arduino connection made" << endl << endl;

  // Outputs status of gamepad1
  if (gamepad1.connected())
  {
    cout << " Gamepad 1 connected" << endl;
  }
  else
  {
    cout << " Gamepad 1 NOT connected" << endl;
  }

  // Outputs status of gamepad2
  if (gamepad2.connected())
  {
    cout << " Gamepad 2 connected" << endl;
  }
  else
  {
    cout << " Gamepad 2 NOT connected" << endl;
  }

  // Main while loop repeatedly feeds input to the arduino
  while (true)
  {
    
    // Determines which gamepad is being used
    if (gamepadToggle) 
    {
      activeGamepad = gamepad1;
    }
    else
    {
      activeGamepad = gamepad2;
    }

    // Gets the current gamepad inputs
    activeGamepad.update()

    // Disable the ROV if back button is pressed or the controller disconnects
    if (activeGamepad.getButtonPressed(xButtons.Back) || !activeGamepad.connected())
    {
      disabled = true;
      manual = true;
      waterLeak = false;
    }
    /* We need to determine what button is available on the gamepad to switch to autonomous mode. This could also be done through the drivestation if that is preferable.
    //Switches to autonomous control if the ?? button is pressed
    else if (activeGamepad.getButtonPressed(??) || drivestation.set("autonomous")) // pseudocode
    {
      manual = false;
    }
    */
    // Returns to manual mode and restarts the ROV's operations if the start button is pressed
    else if (activeGamepad.getButtonPressed(xButtons.Start) && activeGamepad.connected())
    {
      disabled = false;
      manual = true;
    }

    // If manual input is enabled, then run the function using the controller inputs
    if (manual) 
    {
      teleop(1);
    }
    // Otherwise run the function using the camera data
    else 
    {
      teleop(0);
    } 

    activeGamepad.refresh();

  }

  cout << " Exiting" << endl;

  return 0;

}
