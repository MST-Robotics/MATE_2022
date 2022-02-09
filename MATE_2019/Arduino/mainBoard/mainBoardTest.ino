// Teensy++ 2.0
// Test Board 

#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include <Servo.h>

#define LSM9DS1_M 0x1E   // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B  // Would be 0x6A if SDO_AG is LOW

#define SerialConnection Serial

Servo TEST;

const int buzzer = 17;
const int water1 = 20;
const int water2 = 21;
const int water3 = 22;
const int water4 = 23;

const int COMMAND_SIZE = 7;
const int MOTOR_NEUTRAL = 1500;

int timer = 0;

String disabledCommand = ":1500;0";

void setup() // this is run when the board starts up
{
  delay(6000); 

  Wire.begin();
  SerialConnection.begin(115200);
  SerialConnection.setTimeout(80);

  TEST.attach(27);

  pinMode(buzzer, OUTPUT);
  pinMode(water4, INPUT);

  digitalWrite(buzzer, 0);
}

void loop() // this is run after setup
{
  static bool wireInit = false;
  while (!wireInit)
  {
    for(int i = 10; i<14; i++) {
      Wire.beginTransmission(i);
      Wire.write("1500");
      Wire.write(':');
      Wire.endTransmission();
    }
    delay(90);
    ++timer;

    if (timer > 50)
    {
      wireInit = true;
    }
  }
  ++timer;
	
  char cstr[16];
  itoa(timer, cstr, 10);

  // Wait untill there is at least 1 full command to read
  if (SerialConnection.available() >= COMMAND_SIZE - 1)
  {
    // Don't read a string that starts in the middle of a command
    if (SerialConnection.read() == ':')
    {
      timer = 0;  // Reset timer if valid data received
	    
      String info;
      info = disabledCommand;

      info = SerialConnection.readStringUntil('\n');
      info.remove(COMMAND_SIZE-1);

      writeString(info);

      digitalWrite(buzzer, 1);

      clearSerial();
    }
    else
    {
      // Clear invalid command
      SerialConnection.readStringUntil('\n');
    }
  }
  delay(1);
  digitalWrite(buzzer, 0);
}

void clearSerial() {
  while(SerialConnection.available())
  {
    SerialConnection.read();
  }
}

// Used to serially push out a String with Serial.write()
void writeString(String stringData)
{
  for (unsigned int i = 0; i < stringData.length(); i++)
  {
    SerialConnection.write(stringData[i]);  // Push each char 1 by 1 on each loop pass
  }
}
