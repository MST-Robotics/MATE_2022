#include <iostream>

#include ".\Headers\Gamepad.h"
#include ".\Headers\PID.h"
#include ".\Headers\SerialPort.h"
#include ".\Headers\Utils.h"





/* CODE FOR TESTING ARDUINO Tx/Rx */






using namespace std;

char output[MAX_DATA_LENGTH]; // This is currently 255

const sleepTime = 90; // Added a new variable to easily change the command sleep time

// Change the name of the port with the port name of your computer
// Must remember that the backslashes are essential so do not remove them
const char* port = "\\\\.\\COM3";
SerialPort arduino(port, 115200);

// Takes different test strings to send/receive back from the Arduino
int main()
{
  const int data_size = 4;
  string test_data[data_size] = {"test", "00000000", "AAAAAAAA", "999999999999"};
  
  // Loops until the arduino is connected
  while (!arduino.isConnected())
  {
    cout << " Error in Arduino port name" << endl << endl;
  }
  cout << " Arduino connection made" << endl << endl;

  for (int i = 0; i < data_size; i++) 
  {
    cout << " Sending: " << test_data[i];

    char* charArray = new char[test_data[i].size()];
    copy(test_data[i].begin(), test_data[i].end(), charArray);
    
    // Send command array to arduino
    arduino.writeSerialPort(charArray, data.size() - 1);
    delete[] charArray;

    // Wait for arduino to process command
    Sleep(sleepTime); // This value will need to be adjusted as we test the ROV (how long each command typically takes)

    // Expects to receive the command string echoed back, unless there is a water leak, then it will just recieve 'W'
    arduino.readSerialPort(output, MAX_DATA_LENGTH);

    // The loop will output each of the characters in the output array
    cout << " Received: ";
    for (char c : output)
    {
      cout << c;
    }
    cout << endl;
  }
  cout << " Exiting" << endl;

  return 0;

}
