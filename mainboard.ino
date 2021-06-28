// Teensy++ 2.0

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

#define SerialConnection Serial

#define COMMAND_SIZE 52

int timer = 0;

String disabledCommand = ":1500;1500;1500;1500;1500;1500;1500;1500;1500;1500;0";

void setup()
{
  delay(6000); 

  Wire.begin();
  SerialConnection.begin(115200);
  SerialConnection.setTimeout(80); //80?
  
  // This section is designed to give the i2c connection time to initialize (maybe unnecessary?)
  while (timer < 50)
  {
    // Note: talk about i2c addresses 
    for (byte i = 10; i<13; i++) 
    {
      Wire.beginTransmission(i);
      Wire.write("1500");
      Wire.write("1500");
      Wire.write(':');
      Wire.endTransmission();
      delay(90);
      ++timer;
    }
  }
}

void loop()
{
  char driveCommands[COMMAND_SIZE];
  ++timer;

  // Wait untill there is at least 1 full command to read
  if (SerialConnection.available() > COMMAND_SIZE)
  {
    // Don't read a string that starts in the middle of a command
    if (SerialConnection.read() == ':')
    {
      timer = 0;  // Reset timer if valid data received

      String info;
      info = disabledCommand;
      info = SerialConnection.readStringUntil('\n');
      info.remove(COMMAND_SIZE-1);
      info.toCharArray(driveCommands, COMMAND_SIZE - 1);
      drive(driveCommands);
      
      writeString(info);
      
      clearSerial();
    }
    else
    {
      // Clear invalid command
      SerialConnection.readStringUntil('\n');
    }
  }
  
  // Set motors to neutral if no command has been received within 25 loop() cycles
  if (timer > 25)
  {
    disabledCommand.toCharArray(driveCommands, COMMAND_SIZE - 1); 
    drive(driveCommands);
  }
  //Rough timer counting (Ensure counter is at least one ms)
  delay(1);
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
  for (char c : stringData)
    SerialConnection.write(c);  // Push each char 1 by 1 on each loop pass
}

void drive(char array[])
{
  char *commands[35];
  char *ptr = NULL;
  byte index = 0;
  ptr = strtok(array, ";");
  while (ptr != NULL)
  {
    commands[index] = ptr;
    index++;
    ptr = strtok(NULL, ";");
  }
  for (byte addr=10, pos=0; addr<14; pos+=2) {
    Wire.beginTransmission(addr++);
    Wire.write(commands[addr]);
    Wire.write(commands[addr+1]);
    Wire.write(':');
    byte b = Wire.endTransmission();
    if (b != byte(0)) {
      // debug: SerialConnection.write("i2c error: " + b);
    }
  }
}
