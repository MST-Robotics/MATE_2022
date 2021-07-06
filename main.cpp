#include <iostream>

//Pull the files that connects peripherals, laptop, and ROV
#include ".\Headers\Gamepad.h"
#include ".\Headers\PID.h"
#include ".\Headers\SerialPort.h"
#include ".\Headers\Utils.h"

using namespace std;

bool killCode = false;
//bool waterFound = false;

string port;
SerialPort arduino;
char output[MAX_DATA_LENGTH];
//ROV values
int pitch = 0;
int roll = 0;
int yawOffset = 0;

//IMU control variables
string imu;
string prevIMU;
double pitchAutocorrect = 0.0;
double rollAutocorrect = 0.0;

Gamepad gamepad1 = Gamepad(1);
Gamepad gamepad2 = Gamepad(2);

void transferData(string data)
{
  //Change commands into char array
  char* charArray = new char[data.size()];
  copy(data.begin(), data.end(), charArray);
  //Send commands to ROV
  arduino.writeSerialPort(charArray, data.size() - 1);
  delete[] charArray;
  
  Sleep(90);
  
  arduino.readSerialPort(output, MAX_DATA_LENGTH);
  cout << "Recieved: ";
  
  //Print recieved string into console 
  for (char c : output)
  {
    cout << c;
  }
  //For WaterLeak test, search particular position for '1'
}

void teleop(double FWD, double STR, double RCW)
{
  string data = ":";
  
  //IMU control, not used yet
  //Allows driver to adjust correction
  if (gamepad1.getButtonPressed(xButtons.A))
  {
    pitchAutocorrect += -0.01;
  }
  else if (gamepad1.getButtonPressed(xButtons.Y))
  {
    pitchAutocorrect += 0.01;
  }

  if (gamepad1.getButtonPressed(xButtons.B))
  {
    rollAutocorrect += -0.01;
  }
  else if (gamepad1.getButtonPressed(xButtons.X))
  {
    rollAutocorrect += 0.01;
  }
  
  double heading = -0.7853975; //based on 45 degree angle of motors
  //!!!VALUES AND MOTORS MAY CHANGE BASED ON NEW ROV DESIGN!!!
  double FR = (-STR * sin(heading) - FWD * sin(heading) + RCW);   // A
  double BR = (STR * cos(heading) + FWD * sin(heading) - RCW);    // B
  double BL = (-STR * sin(heading) + FWD * cos(heading) - RCW);  // C
  double FL = (-STR * cos(heading) + FWD * cos(heading) - RCW);    // D

  double UL = -(gamepad1.rightTrigger() - gamepad1.leftTrigger());
  double UR = -(gamepad1.rightTrigger() - gamepad1.leftTrigger());
  double UB = -(gamepad1.rightTrigger() - gamepad1.leftTrigger()) * 0.4;
  
  //Arm values
  double shoulder = 0.0;
  double wristTilt = 0.0;
  double wristTwist = 0.0;
  
  //Update arm values
  shoulder += gamepad2.leftStick_Y() * 0.05;
  wristTilt += gamepad2.rightStick_Y() * 0.05;
  wristTwist += gamepad2.rightStick_X() * 0.5;
  
  double* vals[] = {&FR, &FL, &BL, &BR, &UL, &UR, &UB, &shoulder, &wristTilt, &wristTwist};
  
  //Normalize horizontal motor values if power level goes above 1.0 (100%)
  //Necessary for converting to motor-readable values
  double max = 1.0;
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
  //Normalize vertical motor values
  //Max value doesn't get reset to 1? Kinda sus
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
  //Send nothing if values are below 10% max power or if ROV is disabled
  for (double* num : vals)
  {
    if (abs(*num) < 0.1 || killCode)
    {
      *num = 0.0;
    }
  }
  //Convert values into motor-readable numbers and add to data string
  for (double* num : vals)
  {
    *num = Utils::convertRange(-1.0, 1.0, 1100.0, 1900.0, *num);
    data.append(to_string((int)*num) + ";");
  }
  //Add claw command to data string
  data.append(to_string((int)gamepad1.getButtonPressed(xButtons.A)) + "\n");
  
  cout << " Sending: " << data;
  transferData(data);
}

int main()
{
  //Look for connected port on computer and run commands through that port
  //Will loop until it finds the connected port
  cout << "Attempting to connect" << endl;
  for (int i = 0; !arduino.isConnected();)
  {
    port = R"(\\.\COM)" + to_string(i++);

    arduino.openSerialPort(port.c_str(), 115200);
    if (i > 15)
    {
      i = 0;
    }
  }
  cout << "Arduino connected" << endl << endl;
  
  if (gamepad1.connected())
    cout << " Gamepad 1 connected" << endl;
  else
    cout << " Gamepad 1 NOT connected" << endl;
  if (gamepad2.connected())
    cout << " Gamepad 2 connected" << endl;
  else
    cout << " Gamepad 2 NOT connected" << endl;
  
  while (true)
  {
    gamepad1.update();
    gamepad2.update();
    if (gamepad1.getButtonPressed(xButtons.Back) || !gamepad1.connected())
    {
      killCode = true;
      //waterFound = false; //resets water sensor when ROV is restarted
    }
    else if (gamepad1.getButtonPressed(xButtons.Start) && gamepad1.connected())
    {
      killCode = false;
    }
    teleop(gamepad1.leftStick_Y(), gamepad1.leftStick_X(),
           gamepad1.rightStick_X());
    gamepad1.refresh();
    gamepad2.refresh();
}
