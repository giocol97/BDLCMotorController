/*#include <Arduino.h>
#include <SimpleFOC.h>

// DRV8302 pins connections
// don't forget to connect the common ground pin
#define INH_A 21
#define INH_B 19
#define INH_C 18
#define EN_GATE 5
#define M_PWM 25
#define M_OC 26
#define OC_ADJ 12
#define OC_GAIN 14
#define IOUTA 34
#define IOUTB 35
#define IOUTC 32
// Motor instance
BLDCMotor motor = BLDCMotor(15);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// DRV8302 board has 0.005Ohm shunt resistors and the gain of 12.22 V/V
LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

// encoder instance
Encoder encoder = Encoder(22, 23, 1024);
// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {
  Serial.begin(115200);
  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  // DRV8302 specific code
  // M_OC  - enable overcurrent protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum overcurrent limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);
  pinMode(OC_GAIN,OUTPUT);
  digitalWrite(OC_GAIN,LOW);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 15000;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  cs.linkDriver(&driver);
  // align voltage
  motor.voltage_sensor_align = 1.0;
  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.motion_downsample = 0.0;
  // velocity loop PID
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 5.0;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 20.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID
  motor.PID_current_q.P = 1.0;
  motor.PID_current_q.I = 200.0;
  // Low pass filtering time constant
  motor.LPF_current_q.Tf = 0.01;
  // current d loop PID
  motor.PID_current_d.P = 1.0;
  motor.PID_current_d.I = 200.0;
  // Low pass filtering time constant
  motor.LPF_current_d.Tf = 0.01;
  // Limits
  motor.velocity_limit = 15.0; // 100 rad/s velocity limit
  motor.voltage_limit = 23.0;   // 12 Volt limit
  motor.current_limit = 10.0;    // 2 Amp current limit
  // use monitoring with serial for motor init
  // initialise motor
  motor.init();

  // including this line causes the ESP32 to crash:
  cs.init();
}

void loop() {
  // Very simply trying to read pin 4 and print the value to the console.
  float num = analogRead(4);
  Serial.println(num);
}*/