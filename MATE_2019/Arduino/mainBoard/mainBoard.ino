// Teensy++ 2.0

// Other libraries used
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>

// This allows the programmer to give a name to a constant value before the program is compiled, so that memory space is not used during runtime
#define LSM9DS1_M 0x1E   // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B  // Would be 0x6A if SDO_AG is LOW

// The Serial Connection is used to communicate with the laptop
#define SerialConnection Serial

// The inertial measurement unit (IMU) is used to take measurements of the current position of the ROV
LSM9DS1 imu;

// These servos control the various directional motors on the ROV
Servo FR;
Servo BR;
Servo BL;
Servo FL;
Servo UL;
Servo UR;
Servo UB;

// These pertain to the claw action
Servo shoulderTilt;
Servo wristTilt;
Servo wristTwist;

// These constants are used to send data to specific pins
const int buzzer = 17;
const int water1 = 20;
const int water2 = 21;
const int water3 = 22;
const int water4 = 23;

const int COMMAND_SIZE = 52; // The command size is the length of the string sent over serial connection
const int MOTOR_NEUTRAL = 1500; // This is when the motor will be still

int timer = 0; // A timer is used to track the time between receiving commands

String disabledCommand = ":1500;1500;1500;1500;1500;1500;1500;1500;1500;1500;0"; // This command would return all motors to the neutral state

void setup() // The setup function will only run once, after each powerup or reset of the Arduino board. It initialize variables, pin modes, and other information.
{
  
  delay(6000); // Wait for everything to power up

  Wire.begin();
  SerialConnection.begin(115200); // Establishes a serial connection with a baud rate of 115200
  SerialConnection.setTimeout(80); // Sets the maximum milliseconds to wait for serial data

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
	
	// This maps the servos to their specific pins
  FR.attach(27);
  BR.attach(14);
  BL.attach(15);
  FL.attach(16);

  UL.attach(25);
  UR.attach(24);
  UB.attach(26);

//  shoulderTilt.attach(9);
//  wristTilt.attach(10);
//  wristTwist.attach(11);

  pinMode(buzzer, OUTPUT);
  pinMode(water4, INPUT);

  // Scream untill IMU connected
  //digitalWrite(buzzer, 1);

  while (!imu.begin()) // Initializes the IMU
  {
    delay(500);
  }

  digitalWrite(buzzer, 0); // Stops the noise once it is successfully setup
}

void loop() // The loop function continuously runs after the setup function has been completed, it is essentially the main function with built-in looping
{
  static bool wireInit = false;
  while (!wireInit) // Initializing the wire
  {
    for(int i = 10; i<14; i++) {
      Wire.beginTransmission(i);
      Wire.write("1500");
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
  char driveCommands[COMMAND_SIZE]; // These are the commands that will be sent to the motors

  ++timer;

  float ax = 0.0;
  float ay = 0.0;
  float az = 0.0;

  if (imu.accelAvailable()) // Uses the IMU to determine acceleration in three dimensions
  {
    // Updates ax, ay, and az
    imu.readAccel();
    ax = imu.ax;
    ay = imu.ay;
    az = imu.az;
  }

  //if (imu.gyroAvailable())
  {
    // Updates gx, gy, and gz
    //imu.readGyro();
  }

  float roll = atan2(ay, az) * 180 / PI; // calculates the roll of the ROV
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI; // calculates the pitch of the ROV

  char cstr[16];
  itoa(timer, cstr, 10); // converts the int timer to a string cstr

  // Wait until there is at least 1 full command to read
  if (SerialConnection.available() >= COMMAND_SIZE - 1)
  {
    // Don't read a string that starts in the middle of a command
    if (SerialConnection.read() == ':')
    {
	    timer = 0;  // Reset timer if valid data received

      // Only send data back if data was received
//      if (digitalRead(water4) && digitalRead(water3) && digitalRead(water2) && digitalRead(water1))
//      {
//        writeString("0");
//      }
//      else
//      {
//        writeString("1");
//      }
      String info; // will be used to build the command string
      info = disabledCommand;

      info = SerialConnection.readStringUntil('\n');
      info.remove(COMMAND_SIZE-1);
      info.toCharArray(driveCommands, COMMAND_SIZE - 1); // transfers the command to driveCommands
	    
      drive(driveCommands); // sends the commands to the proper motors

      writeString(info); // returns the command back to the laptop over serial

      digitalWrite(buzzer, 1); // makes a noise

      clearSerial(); // clears the serial buffer
    }
    else
    {
      // Clear invalid command
      SerialConnection.readStringUntil('\n');
    }
  }
  
  // Only run if a command has been received within 25 ticks
  if (timer > 25)
  {
    disabledCommand.toCharArray(driveCommands, COMMAND_SIZE - 1); 
    drive(driveCommands);
  }
  //Rough timer counting
  delay(1);
  digitalWrite(buzzer, 0); // stops making noise
}

// clears out the serial read buffer
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

// Routes the different command components to the proper motors
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

  Wire.beginTransmission(10);
  Wire.write(commands[0]);
  Wire.write(commands[3]);
  Wire.write(':');
  Wire.endTransmission();

  Wire.beginTransmission(11);
  Wire.write(commands[2]);
  Wire.write(commands[1]);
  Wire.write(':');
  Wire.endTransmission();

  Wire.beginTransmission(12);
  Wire.write(commands[4]);
  Wire.write(commands[5]);
  Wire.write(':');
  Wire.endTransmission();

  Wire.beginTransmission(13);
  Wire.write(commands[6]);
  Wire.write(commands[7]);
  Wire.write(':');
  Wire.endTransmission();
}
