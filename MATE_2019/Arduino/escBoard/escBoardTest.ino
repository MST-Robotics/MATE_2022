// Testboard code for echoing commands

// Arduino Nano
// ATmega328P (Old Bootloader)

#include <Servo.h>
#include <Wire.h>

Servo M0;

const int MOTOR_NEUTRAL = 1500;

int commandM0 = MOTOR_NEUTRAL;
  
void setup() 
{
  int i2c_addr = 10;
  Wire.begin(i2c_addr);
  Wire.onReceive(receiveEvent);

  pinMode(PD2, OUTPUT);
  pinMode(PD3, OUTPUT); //STAT0
  pinMode(PD4, OUTPUT); //STAT1
  pinMode(PD7, OUTPUT); // Enable
  digitalWrite(PD7, 1);
  pinMode(PD6, OUTPUT); // Voltage Set
  analogWrite(PD6, 100);
  
  M0.attach(PD2);

}

void loop() 
{
  static long lastTime = millis();
  if(millis()-lastTime > 500)
  {
    lastTime = millis();
    digitalWrite(PD4,!digitalRead(PD4));
  }
  M0.writeMicroseconds(commandM0);
}

// Expecting WXYZ:, WXYZ goes to M0
void receiveEvent(int howMany)
{
  digitalWrite(PD3, 1);
  char commandInput[5] = {"1500:"};
  char command1[4] = {"1500"};
  int x = 0;
  
  while (Wire.available() && x<5)
  {
    commandInput[x++] = Wire.read();
  }

  if (commandInput[4] == ':')
  {
    for (int i = 0; i < 4; ++i)
    {
      command1[i] = commandInput[i];
    }
    
    for (int i = 0; i < 9; ++i)
    {
      commandInput[i] = '0';
    }
    
    commandM0 = 1000*(command1[0]-'0')+100*(command1[1]-'0')+10*(command1[2]-'0')+(command1[3]-'0');
    
    digitalWrite(PD3, 0);
  }
  
}
