#include <SimpleFOC.h>
#include "headers.h"

HardwareSerial logSerial = Serial1;

//#define COMMANDER_ENABLED

// Hall sensor instance
// HallSensor(int hallA, int hallB , int hallC , int pp)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(HALL_U, HALL_V, HALL_W, POLE_PAIRS); // TODO

//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC)
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_U, PWM_V, PWM_W); // TODO

// current sense?

//  BLDCMotor( pole_pairs , ( phase_resistance, KV_rating  optional) )
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING); //, 220); // TODO

Commander command = Commander(logSerial);

void doMotor(char *cmd) { command.motor(&motor, cmd); }

//  LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
LowsideCurrentSense current_sense = LowsideCurrentSense(SHUNT_RESISTOR, CURRENT_SENSING_GAIN, I_U, I_V, I_W);

TaskHandle_t TaskHandleSpeed;
TaskHandle_t TaskHandleData;

float currentAngle = 0;
float currentSpeed = 0;

void sensorA() { sensor.handleA(); }
void sensorB() { sensor.handleB(); }
void sensorC() { sensor.handleC(); }

void initPins()
{
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

 /* pinMode(I_U, INPUT);
  pinMode(I_V, INPUT);
  pinMode(I_W, INPUT);*/

  analogReadResolution(10);
  analogSetAttenuation(ADC_0db);

 /*pinMode(HALL_U, INPUT_PULLUP);
  pinMode(HALL_V, INPUT_PULLUP);
  pinMode(HALL_W, INPUT_PULLUP);*/

 /* pinMode(PWM_U, OUTPUT);
  pinMode(PWM_V, OUTPUT);
  pinMode(PWM_W, OUTPUT);*/
}

void setupa()
{

  logSerial.begin(LOG_BAUD, SERIAL_8N1, LOG_RX, LOG_TX);
  logSerial.println("Starting new");

  delay(1000);

  initPins();

  // initialize sensor hardware
  logSerial.println("Initializing sensors");
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(sensorA, sensorB, sensorC);
  logSerial.println("Sensors OK");

  // pwm frequency to be used [Hz]
  logSerial.println("Initializing driver");
  driver.pwm_frequency = PWM_FREQUENCY;
  // power supply voltage [V]
  driver.voltage_power_supply = 16.8; // 14.4;
  // Max DC voltage allowed - default voltage_power_supply
  // driver.voltage_limit = 16.8;
  // driver init
  driver.init();
  logSerial.println("Driver OK");

  current_sense.linkDriver(&driver);

  logSerial.println("Initializing commander");
  command.add('M', doMotor, "motor");

  // use monitoring with the BLDCMotor
  // monitoring port
  logSerial.println("Initializing motor");
  motor.useMonitoring(logSerial);

  // init sensor
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // init driver
  // link the motor to the driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.torque_controller = TorqueControlType::foc_current;
  // motor.torque_controller = TorqueControlType::voltage;
  logSerial.printf("Torque control type: %d \n", motor.torque_controller);
  motor.controller = MotionControlType::torque;

  motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 1000;
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002f; // 1ms default
  motor.LPF_current_d.Tf = 0.002f; // 1ms default

  motor.voltage_limit = 16.8; // 16.8;
  motor.current_limit = 4.3;
  motor.velocity_limit = 100; //rad/s

  motor.foc_modulation = FOCModulationType::SinePWM;
  // initialize motor
  motor.init();

  delay(100);

  // init current sense
  if (current_sense.init())
    logSerial.println("Current sense init success!");
  else
  {
    logSerial.println("Current sense init failed!");
    return;
  }

  // link the motor to current sense
  motor.linkCurrentSense(&current_sense);

  /*current_sense.gain_a = -5.0f;
  current_sense.gain_b = -5.0f;
  current_sense.gain_c = -5.0f;*/

  current_sense.skip_align = true;

  logSerial.println("Motor pre FOC OK");

  delay(100);

  motor.initFOC();
  logSerial.println("Motor OK");

  logSerial.println("Done configuring");

  delay(1000);

  // motor.target = (300);

  //#ifndef COMMANDER_ENABLED
  /*xTaskCreatePinnedToCore(
      TaskPrintData,
      "TaskLoop",
      5000,
      NULL,
      5,
      &TaskHandleData,
      0);

  xTaskCreatePinnedToCore(
      TaskControlSpeed,
      "TaskSpeed",
      5000,
      NULL,
      5,
      &TaskHandleSpeed,
      1);*/

  //#endif
}

void TaskControlSpeed(void *pvParameters)
{
  float target_speeds[6] = {20.0, 160.0, 300.0, 380.0, 250.0, 80.0};
  float target_currents[6] = {0.2, 0.8, 1.5, 2.0, 1.0, 0.5};

  while (1)
  {
    /*for (int i = 0; i < 6; i++)
    {
      logSerial.print("Setting speed to ");
      logSerial.print(target_speeds[i]);
      logSerial.println(" rad/s");
      motor.target = (target_speeds[i]);
      vTaskDelay(3000);
    }*/
    for (int i = 0; i < 6; i++)
    {
      logSerial.print("Setting current to ");
      logSerial.print(target_currents[i]);
      logSerial.println(" A");
      motor.target = (target_currents[i]);
      // motor.current_sp = (target_currents[i]);
      //  motor.target = 0.005;

      vTaskDelay(5000);
    }
  }
  // motor.target=(3.0 * -3.14);
}

void TaskPrintData(void *pvParameters)
{

  int curA, curB, curC = 0;

  while (1)
  {

    /*currentSpeed=sensor.getVelocity();
    currentAngle=sensor.getAngle();
     logSerial.printf("Motor - Speed: %f rad/s, Angle: %f rad\n" ,
                   currentSpeed, currentAngle );*/

    logSerial.printf("voltages to phases: %f, %f, %f\n", motor.Ua, motor.Ub, motor.Uc);

    /* for (int i = 0; i < 100; i++)
     {
       curA += analogRead(32);
       curB += analogRead(33);
       curC += analogRead(25);
       delay(1);
     }

     curA /= 100;
     curB /= 100;
     curC /= 100;*/

    PhaseCurrent_s currents;
    currents = current_sense.getPhaseCurrents();
    float current_mag = current_sense.getDCCurrent();

    // logSerial.printf("Gains: %f, %f, %f\n", current_sense.gain_a, current_sense.gain_b, current_sense.gain_c);

    logSerial.printf("Currents - A: %.2f, B: %.2f, C: %.2f, DC: %.2f\n", currents.a * 1000, currents.b * 1000, currents.c * 1000, current_mag * 1000);

    DQCurrent_s currentsFOC = current_sense.getFOCCurrents(currentAngle);

    logSerial.printf("FOC currents - D: %f, Q: %f\n", currentsFOC.d, currentsFOC.q);

    // logSerial.printf("Analog reads: %d, %d, %d\n", analogRead(32), analogRead(33), analogRead(25));
    //   logSerial.printf("READING CURRENTS: %d %d %d\n", curA, curB, curC);

    delay(1000);
  }
}

void loopa()
{
  // FOC algorithm function
  motor.loopFOC();
  motor.move();

  motor.monitor();
  command.run();

  // motor.move();

  //currentAngle = sensor.getAngle();
  //currentSpeed = sensor.getVelocity();
}