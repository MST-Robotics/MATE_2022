// Teensy++ 2.0
/*  If you have a clean arduino install there are two additional libs you will
     need to install using the Tools>Manage Libraries;  Sparkfun LSM9DS1 IMU and
     SensorFusion. Once that's done look in your arduino libraries folder for examples
     For an in-depth guide, check out https://learn.sparkfun.com/tutorials/lsm9ds1-breakout-hookup-guide
*/
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <SensorFusion.h>

#define SerialConnection Serial
#define COMMAND_SIZE 52
#define MOTOR_NEUTRAL 1500
String disabledCommand = ":1500;1500;1500;1500;1500;1500;1500;1500;1500;1500;0";

#define LSM9DS1_M 0x1E   // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B  // Would be 0x6A if SDO_AG is LOW
LSM9DS1 imu;
SF imuFusion;
const byte imuReadingNum = 5;
float imuReadings[imuReadingNum * 2];
byte imuReadIndex = 0;
float pitchTotal = 0.0f;
float rollTotal = 0.0f;

byte loops = 0;

const byte buzzer = 17;

int timer = 0;


//  Runs once upon startup
void setup()
{
  for (byte i = 0; i < imuReadingNum * 2; ++i)
    imuReadings[i] = 0.0f;

  delay(1000);

  Wire.begin();
  SerialConnection.begin(115200);
  SerialConnection.setTimeout(80);
  while (timer < 50)
  {
    /*  Bytes in arduino are unsigned ints (0-255), essentially smaller int, which helps with limited ram
             The 'i' parameter is the inter-integrated circuit protocol (i2c) address that is being written to. Each component needs a unique address.
    */
    for (byte addr = 10; addr < 14; addr++)
    {
      Wire.beginTransmission(addr);
      //  As per the arduino documentation of Wire.h, any transmission larger than 32 bytes will lose data
      Wire.write("1500");
      Wire.write("1500");
      Wire.write(':');
      Wire.endTransmission();
      delay(90);
      ++timer;
    }
  }
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  // Scream untill IMU connected
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, 1);
  while (!imu.begin())
  {
    delay(100);
  }
  digitalWrite(buzzer, 0);
}

//  Loop runs indefinitely (like any function, non-static variables which fall out of scope are cleared after each loop);
void loop()
{
  char driveCommands[COMMAND_SIZE];
  ++timer;
  float ax = 0.0, ay = 0.0, az = 0.0, gx = 0.0, gy = 0.0, gz = 0.0, mx = 0.0, my = 0.0, mz = 0.0, deltat = 0.0;

  if (imu.accelAvailable())
  {
    // Updates Accelerometer data
    imu.readAccel();
    ax = imu.ax;//imu.calcAccel(imu.ax);
    ay = imu.ay;//imu.calcAccel(imu.ay);
    az = imu.az;//imu.calcAccel(imu.az);
  }

  if (imu.gyroAvailable())
  {
    // Updates Gyroscope data
    imu.readGyro();
    gx = imu.calcGyro(imu.gx) * DEG_TO_RAD;
    gy = imu.calcGyro(imu.gy) * DEG_TO_RAD;
    gz = imu.calcGyro(imu.gz) * DEG_TO_RAD;
  }
/* We currently don't need mag values, since our goal is speed
  if (imu.magAvailable())
  {
    // Updates Compass data
    imu.readMag();
    mx = imu.mx * 0.1;
    my = imu.my * 0.1;
    mz = imu.mz * 0.1;
  }
*/

  deltat = imuFusion.deltatUpdate();
  imuFusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate (for more info, check the SensorFusion ex)

  //  So we reset our pitch total each time by subtracting what we had added in hte previous loop, then adding the new modified value? Why not 
  //  just have pitchTotal be a local variable, Then add Following math is sus. 
  pitchTotal -= imuReadings[imuReadIndex];
  imuReadings[imuReadIndex] = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;//imuFusion.getPitch();
  pitchTotal += imuReadings[imuReadIndex];

  rollTotal -= imuReadings[imuReadIndex + 1];
  imuReadings[imuReadIndex + 1] = atan2(ay, az) * RAD_TO_DEG; //imuFusion.getRoll();
  rollTotal += imuReadings[imuReadIndex + 1];

  imuReadIndex += 2;
  if (imuReadIndex >= imuReadingNum * 2)
  {
    imuReadIndex = 0;
  }

  float pitch = pitchTotal / imuReadingNum;
  float roll = rollTotal / imuReadingNum;

  //float pitch = imuFusion.getPitch();
  //float roll = imuFusion.getRoll();

  //float roll = atan2(ay, az) * RAD_TO_DEG;
  //float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  ++loops;

  char cstr[16];
  itoa(timer, cstr, 10);

  // Wait untill there is at least 1 full command to read
  if (SerialConnection.available() >= COMMAND_SIZE - 1)
  {
    // Don't read a string that starts in the middle of a command
    if (SerialConnection.read() == ':')
    {
      timer = 0;  // Reset timer if valid data received
      // Only send data back if data was received

      String info;
      info = disabledCommand;

      info = SerialConnection.readStringUntil('\n');
      info.remove(COMMAND_SIZE - 1);
      info.toCharArray(driveCommands, COMMAND_SIZE - 1);
      drive(driveCommands);

      // Send back IMU data
      writeString(String(pitch, 2)+";"+String(roll, 2)+";"+String(loops));
      //  Yaw?
      loops = 0;

      clearSerial();
    }
    else
    {
      // Clear invalid command
      SerialConnection.readStringUntil('\n');
    }
  }

  // Only run if a command has been received within 100 ticks
  /*if (timer > 100)
    {
    disabledCommand.toCharArray(driveCommands, COMMAND_SIZE - 1);
    drive(driveCommands);
    }*/
  //Rough timer counting
  delay(1);
}

void clearSerial() {
  while (SerialConnection.available())
  {
    SerialConnection.read();
  }
}

// Used to serially push out a String with Serial.write()
void writeString(String stringData)
{
  for (char c : stringData)
  {
    SerialConnection.write(c);  // Push each char 1 by 1 on each loop pass
  }
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
  for (byte addr = 10, pos = 0; addr < 14; pos += 2) {
    Wire.beginTransmission(addr++);
    Wire.write(commands[addr]);
    Wire.write(commands[addr + 1]);
    Wire.write(':');
    byte b = Wire.endTransmission();
    if (b != byte(0)) {
      // debug: SerialConnection.write("i2c error: " + b);
    }
  }
}
