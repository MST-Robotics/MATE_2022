#include <iostream>
#include <string>
#include ".\Headers\Gamepad.h"
#include ".\Headers\PID.h"
#include ".\Headers\SerialPort.h"
#include ".\Headers\Utils.h"


/* CODE FOR TESTING ARDUINO Tx/Rx */


using namespace std;

char output[MAX_DATA_LENGTH]; // This is currently 255
char out[MAX_DATA_LENGTH];

const int sleepTime = 90; // Added a new variable to easily change the command sleep time

// Change the name of the port with the port name of your computer
// Must remember that the backslashes are essential so do not remove them
const char* port = "\\\\.\\COM3";
SerialPort arduino(port, 115200);

char* transferData(string data)
{
  // Convert data string to character array
  char* charArray = new char[data.size()];
  copy(data.begin(), data.end(), charArray);
  // Send command array to arduino
  arduino.writeSerialPort(charArray, data.size() - 1);
  delete[] charArray;

  // Wait for arduino to process command
  Sleep(sleepTime); // This value will need to be adjusted as we test the ROV (how long each command typically takes)

  // Expects to receive the command string echoed back, unless there is a water leak, then it will just recieve 'W'
  arduino.readSerialPort(output, MAX_DATA_LENGTH);
  
  return output;
}

// Takes different test strings to send/receive back from the Arduino
int main()
{
  // const int data_length = ?; used if we pre-input test strings
  // string *test_data[data_length] = {};
  string test_data;
  
  // Loops until the arduino is connected
  while (!arduino.isConnected())
  {
    cout << " Error in Arduino port name" << endl << endl;
  }
  cout << " Arduino connection made" << endl << endl;

  while (true)
  {
    cout << "Input info: ";
    getline(cin, test_data);
    cout << " Sending: " << test_data << endl;
    
    out = transferData(test_data);

    // The loop will output each of the characters in the output array
    cout << " Received: ";
    for (char c : out)
    {
      cout << c;
    }
    cout << endl;
  }
  
  cout << " Exiting" << endl;

  return 0;

}
