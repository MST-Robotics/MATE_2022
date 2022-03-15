// Teensy 4.0
// Purpose of branch: The purpose of this Teensy code is to test our system at a barebones level. 
// This particular code has the teensy sending preset data to the escs via the I2C data line (SDA,SCL,GND). 
// If running correctly, Teensy light should blink, and values should be send after ~10 seconds.

// Input: 
// If running with PCCOM 0, then no input is required. 
// If running with PCCOM 1, then valid input is a 40 character string 
// Example: :1400;1400;1400;1400;1400;1400;1400;1400
// NOTE: Set serial monitor to 115200 baud and 'New Line' to recieve and send data as expected

// Behavior:
// In PCCOM 0 Mode, sends a default 1400 microseconds to every motor, relays actions through serial.
// Blinks LED twice a second
// In PCCOM 1 Mode, waits for input from the serial port as defined in the 'Input' section, then sends 
// those commands to the motors. Relays actions through serial. Blinks LED when data is recieved.


//#include <Wire.h> //will still be needed for imu eventually
#include <SPI.h>
#include <PWMServo.h>

#define SerialConnection Serial
#define LED 13
#define COMMAND_SIZE 40 //don't forget to count the colon on the front
#define DISABLED_TIME 250 //Max time after a command before the robot shuts off

#define PCCOM 1 //If defined as 1, will expect incomming communication from serial
//If defined as 0, will use default value 1400 us as commands

//UFR, UFL, UBR, UBL, DFR, DFL, DBR, DBL
#define disabledCommand "1500;1500;1500;1500;1500;1500;1500;1500"
PWMServo motors[8];
constexpr byte motorpins[8] = {0,1,2,3,4,5,6,7}; //aligned with motors[]

inline void writeMicroseconds(int usec, int pin) { //override normal implementation with a microsecond one
  //Serial.printf("angle=%d, usec=%.2f, us=%.2f, duty=%d, min=%d, max=%d\n", //angle, usec, (float)us / 256.0f, duty, min16<<4, max16<<4);
  
  //this is the max bit resolution (100% duty cycle) over the period of a 50 Hz signal
  constexpr float pwmmulti = 4096.0f/20000.0f; 
  uint32_t duty = (int)(usec * pwmmulti);
  analogWriteResolution(12);
  analogWrite(pin, duty);
}

void motorInit() {
  for(int i = 0; i < 8; i++) {
    motors[i].attach(motorpins[i]);
  }
}

void setup()
{
  pinMode(LED,OUTPUT);
  digitalWrite(LED, 1);
  Serial.begin(115200);
  Serial.setTimeout(80);
  motorInit();
  digitalWrite(LED, 0);
  Serial.println("Setup Complete!");
  #if PCCOM
    Serial.println("Running in communication mode\nPlease set baud rate to 115200 and send a\n newline character after each command");
  #else
    Serial.println("Running in default value mode, sending data...");
  #endif
}

void loop()
{
  unsigned long lastTime = millis();
  
  #if PCCOM == 0 
  char driveCommands[] = "1400;1400;1400;1400;1400;1400;1400;1400";
  drive(driveCommands);
  if(millis()-lastTime > 500)
  {
    lastTime = millis();
    digitalWrite(LED,!digitalRead(LED));
  }
  #endif
  
  #if PCCOM == 1
  char driveCommands[COMMAND_SIZE];
  // Wait untill there is at least 1 full command to read
  if (SerialConnection.available() >= COMMAND_SIZE)
  {
    // Don't read a string that starts in the middle of a command
    if (SerialConnection.read() == ':')
    {
      if(millis()-lastTime > 500) // We recieved valid data, flash the light
      {
        lastTime = millis();
        digitalWrite(LED,!digitalRead(LED));
      }
   
      String temp = disabledCommand;
      temp = SerialConnection.readStringUntil('\n');
      //temp.remove(position) //used for taking out other motor commands
      temp.toCharArray(driveCommands, COMMAND_SIZE);
      drive(driveCommands);

      Serial.print("In: ");
      Serial.println(temp);
    }
    else
    {
      // Clear invalid command
      SerialConnection.readStringUntil('\n');
    }
  }
  
  // Only run if a command hasn't been received within DISABLED_TIME ms. Tighten timings if needed
  if(millis()-lastTime > DISABLED_TIME)
  {
    char arr[] = disabledCommand;    
    drive(arr);
  }
  #endif
}

void drive(char* array) 
{
  char *commands[40];
  char *ptr = nullptr;
  byte index = 0;
  ptr = strtok(array, ";");
  Serial.print("Out: ");
  while (ptr != nullptr)
  {
    Serial.print(ptr);
    Serial.print(' ');
    commands[index] = ptr;
    index++;
    ptr = strtok(nullptr, ";");
  }
  int int_commands[8];
  for(int i = 0; i<8; i++) {
    int_commands[i]=1000*(commands[i][0]-'0')+100*(commands[i][1]-'0')+10*(commands[i][2]-'0')+(commands[i][3]-'0');
    writeMicroseconds(int_commands[i], motorpins[i]);
  }
  Serial.println();
}
