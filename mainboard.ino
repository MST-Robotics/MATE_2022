// Teensy++ 2.0

#include <SPI.h>
#include <Servo.h>

#define SerialConnection Serial
#define COMMAND_SIZE 52  // Number of chars in a command

int timer = 0;
Servo servos[7];
String disabledCommand = "1500;1500;1500;1500;1500;1500;1500;1500;1500;1500;0";


//  Runs once upon startup...takes a minimum of 13 seconds
void setup()
{
  delay(6000); // This delay is to ensure that both the esc's and the arduino are on, otherwise they'll miss the initialization command
  
  Serial.begin(115200);
  Serial.setTimeout(80); //80?
  
  // valid pins for PWM are (grouped by their timer) {0}, {1, 24}, {14, 15, 16},  {25, 26, 27}
  servos[0].attach(1); // FR
  servos[1].attach(24); // FL
  servos[2].attach(14); // BL
  servos[3].attach(15); // BR
  servos[4].attach(25); // UL
  servos[5].attach(26); // UR
  servos[6].attach(27); // UB

  // Initialize escs
  for(byte i = 0; i < 7; i++)
    servos[i].writeMicroseconds(1500);
  delay(7000);  
  // Servo shoulder, wristTilt, wristTwist; // use later
}


//  Loop runs indefinitely (like any function, non-static variables which fall out of scope are cleared after each loop);
void loop()
{
  char driveCommands[COMMAND_SIZE];
  ++timer;

  // Wait untill there is at least 1 full command to read
  if (Serial.available() > COMMAND_SIZE)
  {
    // Don't read a string that starts in the middle of a command
    if (Serial.read() == ':')
    {
      timer = 0;  // Reset timer if valid data received

      String info;
      info = disabledCommand;
      info = Serial.readStringUntil('\n');
      info.remove(COMMAND_SIZE - 1);
      info.toCharArray(driveCommands, COMMAND_SIZE - 1);
      drive(driveCommands);
      writeString(info);

      clearSerial();
    }
    else
    {
      // Clear invalid command
      Serial.readStringUntil('\n');
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
  while (Serial.available())
  {
    Serial.read();
  }
}

// Used to serially push out a String with Serial.write()
void writeString(String stringData)
{
  for (char c : stringData)
    Serial.write(c);  // Push each char 1 by 1 on each loop pass
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
  // Valid commands are between 1100 and 1900
  for(byte i = 0; i < 7; i++)
    servos[i].writeMicroseconds(commands[i]);
}
