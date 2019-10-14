#include <QTRSensors.h>

#define RightMotorDefaultSpeed   200
#define LeftMotorDefaultSpeed    200

#define LeftMotorPin        13
#define LeftMotorPwmPin     11
#define RightMotorPin       12
#define RightMotorPwmPin    3

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


// PID control variables
int PIDLastFault = 0;
float Kp = 0;
float Kd = 0;
int LeftMotorPwmVal = 0;
int RightMotorPwmVal = 0;

void setup() {
// put your setup code here, to run once:
delay(2000);
// init qtr sensor class
qtr.setTypeRC();
qtr.setSensorPins((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 10}, SensorCount);
qtr.setEmitterPin(2);

// init motor pins
pinMode(RightMotorPin,OUTPUT);
pinMode(RightMotorPwmPin,OUTPUT);

pinMode(LeftMotorPin,OUTPUT);
pinMode(LeftMotorPwmPin,OUTPUT);

// init test led
digitalWrite(13,HIGH);

// 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
// = ~25 ms per calibrate() call.
// Call calibrate() 400 times to make calibration take about 10 seconds.
for (uint16_t i = 0; i < 400; i++)
{
  qtr.calibrate();
}

}

void loop() {
  // put your main code here, to run repeatedly:

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
}

void MotorControl(int rightMotorPwm, int leftMotorPwm)
{
  if(rightMotorPwm <= 0)
  {
    rightMotorPwm = abs(rightMotorPwm);
    digitalWrite(RightMotorPin, LOW);
    analogWrite(RightMotorPwmPin, rightMotorPwm);
  }
  else
  {
    digitalWrite(RightMotorPin, HIGH);
    analogWrite(RightMotorPwmPin, rightMotorPwm);
  }
  if(leftMotorPwm <= 0)
  {
    leftMotorPwm = abs(leftMotorPwm);
    digitalWrite(LeftMotorPin, LOW);
    analogWrite(LeftMotorPwmPin, leftMotorPwm);
  }
  else
  {
    digitalWrite(LeftMotorPin, HIGH);
    analogWrite(LeftMotorPwmPin, leftMotorPwm);
  }
}
