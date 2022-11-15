/*
#include <Arduino.h>
#include <WiFi.h>

#define ADC_U 32
#define PWM_U 27

#define ADC_V 33
#define PWM_V 14

#define ADC_W 25
#define PWM_W 12

// 500 unità = 1A (U-V)

void setupa()
{
  WiFi.mode(WIFI_OFF);

  Serial1.begin(115200, SERIAL_8N1, 15, 4);
  Serial1.println("ASDASDASD");

  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  pinMode(ADC_U, INPUT);
  pinMode(PWM_U, OUTPUT);
  pinMode(ADC_V, INPUT);
  pinMode(PWM_V, OUTPUT);
  pinMode(ADC_W, INPUT);
  pinMode(PWM_W, OUTPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_0db);

  digitalWrite(PWM_U, LOW);
  digitalWrite(PWM_V, LOW);
  digitalWrite(PWM_W, LOW);

  Serial1.println("Starting");

  digitalWrite(PWM_U, HIGH);

  int readAvg = 0;

  for (int i = 0; i < 1000; i++)
  {
    readAvg += (analogRead(ADC_U));
    delay(1);
  }

  digitalWrite(PWM_U, LOW);

  Serial1.println("Done");

  Serial1.println(readAvg);
  Serial1.println(readAvg / 1000);
}

void loopa()
{
  delay(5000);

  Serial1.println("Starting U");

  digitalWrite(PWM_U, HIGH);

  int readAvg = 0;

  for (int i = 0; i < 1000; i++)
  {
    readAvg += (analogRead(ADC_U));
    delay(1);
  }

  digitalWrite(PWM_U, LOW);

  Serial1.println(readAvg);
  Serial1.println(readAvg / 1000);

  Serial1.println("Done U");

  delay(5000);

  Serial1.println("Starting V");

  digitalWrite(PWM_V, HIGH);

  readAvg = 0;

  for (int i = 0; i < 1000; i++)
  {
    readAvg += (analogRead(ADC_V));
    delay(1);
  }

  digitalWrite(PWM_V, LOW);

  Serial1.println(readAvg);
  Serial1.println(readAvg / 1000);

  Serial1.println("Done V");

  delay(5000);

  Serial1.println("Starting W");

  digitalWrite(PWM_W, HIGH);

  readAvg = 0;

  for (int i = 0; i < 1000; i++)
  {
    readAvg += (analogRead(ADC_W));
    delay(1);
  }

  digitalWrite(PWM_W, LOW);

  Serial1.println(readAvg);

  Serial1.println(readAvg / 1000);

  Serial1.println("Done W");
}*/

#include <SimpleFOC.h>
#include "headers.h"

// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensorTest = HallSensor(HALL_W, HALL_U, HALL_V, POLE_PAIRS);

BLDCDriver3PWM driverTest = BLDCDriver3PWM(PWM_U, PWM_V, PWM_W);

BLDCMotor motorTest = BLDCMotor(POLE_PAIRS);
// Interrupt routine intialisation
// channel A and B callbacks
void doA() { sensorTest.handleA(); }
void doB() { sensorTest.handleB(); }
void doC() { sensorTest.handleC(); }

void setupa()
{

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);
  // monitoring port
  Serial1.begin(LOG_BAUD, SERIAL_8N1, LOG_RX, LOG_TX);

  // check if you need internal pullups
  // sensor.pullup = Pullup::USE_EXTERN;

  // initialise encoder hardware
  sensorTest.init();
  // hardware interrupt enable
  sensorTest.enableInterrupts(doA, doB, doC);

  Serial1.println("Sensor ready");

  driverTest.pwm_frequency = 20000;
  // power supply voltage [V]
  driverTest.voltage_power_supply = 16.8;
  // Max DC voltage allowed - default voltage_power_supply
  // driverTest.voltage_limit = 16.8;


  // driver init
  driverTest.init();

  motorTest.linkDriver(&driverTest);

  motorTest.voltage_limit = 16.8;
  //motorTest.current_limit = 2.5;
  motorTest.controller = MotionControlType::velocity_openloop;

  motorTest.init();


  Serial1.println("driver ready");

  delay(1000);
}

void loopa()
{
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  sensorTest.update();
  // display the angle and the angular velocity to the terminal
  //Serial1.print(sensorTest.getAngle());
  //Serial1.print("\t");
  Serial1.println(sensorTest.getVelocity());

  motorTest.move(6*6.28);
}
