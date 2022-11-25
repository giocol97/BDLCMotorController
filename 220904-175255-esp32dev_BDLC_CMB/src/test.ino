#include <SimpleFOC.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#include "headers.h"

DynamicJsonDocument data(1024);

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

/*Commander command = Commander(logSerial);

void doMotor(char *cmd) { command.motor(&motor, cmd); }*/

//  LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
LowsideCurrentSense current_sense = LowsideCurrentSense(SHUNT_RESISTOR, CURRENT_SENSING_GAIN, I_U, I_V, I_W);

TaskHandle_t TaskHandleSpeed;
TaskHandle_t TaskHandleData;

TaskHandle_t TaskHandle0;
TaskHandle_t TaskHandle1;
TaskHandle_t TaskHandleSerial;

void sensorA() { sensor.handleA(); }
void sensorB() { sensor.handleB(); }
void sensorC() { sensor.handleC(); }

// state variables

uint8_t currentSystemState = STATE_START;
uint8_t prevState = STATE_START;

float currentSpeed = 0;
float currentAngle = 0;
float lastSpeed = 0;

float raw_speed[50] = {0};
float raw_angle[50] = {0};

int sensor_index = 0;

#define UNDEFINED_VALUE -123456
#define RAIL_LENGTH_DEBUG 1400

// web parameters TODO define defaults in header
float vmax = 200;                         // rad/s
float vmin = 50;                          // rad/s
float rampDuration = UNDEFINED_VALUE;     // ms TODO use
int pulseStart = RAIL_LENGTH_DEBUG * 0.1; // pulses
int pulseStop = RAIL_LENGTH_DEBUG * 0.75; // pulses
int pulseEnd = RAIL_LENGTH_DEBUG * 0.9;   // pulses
float tend = 0.75;                        // V
float tbrake = 0.3;
long timeoutDuration = 5000;

int state = 0;

int timeoutStart = 0;
bool closing = false;
bool launchDebug = false;

void initPins()
{
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);

  pinMode(I_U, INPUT);
  pinMode(I_V, INPUT);
  pinMode(I_W, INPUT);

  analogReadResolution(10);
  analogSetAttenuation(ADC_0db);

  pinMode(HALL_U, INPUT_PULLUP);
  pinMode(HALL_V, INPUT_PULLUP);
  pinMode(HALL_W, INPUT_PULLUP);

  pinMode(PWM_U, OUTPUT);
  pinMode(PWM_V, OUTPUT);
  pinMode(PWM_W, OUTPUT);
}

void setup()
{

  WiFi.mode(WIFI_OFF);

  Serial.begin(LOG_BAUD);

  logSerial.begin(LOG_BAUD, SERIAL_8N1, LOG_RX, LOG_TX);
  logSerial.println("Starting new");

  delay(500);

  initPins();

  delay(100);

  // initialize sensor hardware
  logSerial.println("Initializing sensors");
  sensor.pullup = Pullup::USE_INTERN;
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

  motor.voltage_sensor_align = 1.5;

  /*logSerial.println("Initializing commander");
  command.add('M', doMotor, "motor");*/

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
  motor.torque_controller = TorqueControlType::voltage;
  // motor.torque_controller = TorqueControlType::voltage;
  logSerial.printf("Torque control type: %d \n", motor.torque_controller);
  motor.controller = MotionControlType::velocity;

  // controller configuration based on the control type
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0.0;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 50;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.001;

  /*motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 1000;
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002f; // 1ms default
  motor.LPF_current_d.Tf = 0.002f; // 1ms default*/

  motor.voltage_limit = 16.8;
  // motor.current_limit = 4.3;
  motor.velocity_limit = MAX_SPEED; // rad/s

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

  // current_sense.skip_align = true;

  logSerial.println("Motor pre FOC OK");

  delay(100);

  motor.initFOC();
  logSerial.println("Motor OK");

  logSerial.println("Done configuring");

  motor.disable();
  digitalWrite(ENABLE_PIN, LOW);

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

  xTaskCreatePinnedToCore(
      Task0,
      "Task0",
      5000,
      NULL,
      5,
      &TaskHandle0,
      0);

  xTaskCreatePinnedToCore(
      Task1,
      "Task1",
      5000,
      NULL,
      10,
      &TaskHandle1,
      1);

  xTaskCreatePinnedToCore(
      TaskSerial,
      "Taskserial",
      5000,
      NULL,
      9,
      &TaskHandleSerial,
      1);

  //#endif
}

void Task0(void *pvParameters) // task raccolta dati/commander
{
  while (1)
  {
    // logSerial.println("Task 1");
    // motor.monitor();
    // command.run();

    if (sensor_index != 50)
    {
      raw_speed[sensor_index] = sensor.getVelocity();
      raw_angle[sensor_index] = sensor.getAngle();
      sensor_index++;
    }
    else
    {
      sensor_index = 0;
      lastSpeed = currentSpeed;
      currentSpeed = avgNoZero(raw_speed, 50);
      currentAngle = avgNoZero(raw_angle, 50);
    }

    delay(1);
  }
}

float brakeVoltage(float speed)
{
  return map(speed, 50, 200, 0, 5) / 10;
}

bool updateState(int pulses, float speed, int millis)
{
  switch (currentSystemState)
  {
  case STATE_START:
    logSerial.println("Error, invalid state");
    return false;
    break;
  case STATE_INACTIVE:
    if (pulses > pulseStart && speed > vmax)
    {
      currentSystemState = STATE_SPINTA;
    }
    else if (pulses > pulseEnd && speed < vmin)
    {
      currentSystemState = STATE_FINECORSA;
    }
    break;
  case STATE_SPINTA:
    if (pulses > pulseStop)
    {
      currentSystemState = STATE_FRENATA;
    }
    break;
  case STATE_FRENATA:
    if (pulses > pulseEnd && speed == 0)
    { // TODO mettere speed "circa" 0?
      currentSystemState = STATE_FINECORSA;
    }
    break;
  case STATE_FINECORSA:
    if (millis - timeoutStart > timeoutDuration /* || speed < -vmax*/)
    {
      currentSystemState = STATE_RITORNO_VEL;
    }
    break;
  case STATE_RITORNO_VEL:
    if (pulses < pulseStart)
    {
      currentSystemState = STATE_RITORNO_TOR;
    }
    break;
  case STATE_RITORNO_TOR:
    if (speed == 0)
    {
      currentSystemState = STATE_INACTIVE;
    }
    break;
  }
  return true;
}

void Task1(void *pvParameters) // task implementazione funzionalità
{

  while (1)
  {
    delay(50);

    int pulses = radiansToImpulses(currentAngle);

    DQVoltage_s voltage = motor.voltage;

    data["time"] = millis();
    data["angle"] = currentAngle;
    data["pulses"] = pulses;
    data["speed"] = currentSpeed;
    data["voltage"] = voltage.q;
    data["target"] = motor.target;
    data["control"] = motor.controller;
    data["state"] = state;
    // logSerial.println("Current angle: " + String(currentAngle) + " - Current speed: " + String(currentSpeed) + " - Current pulses: " + String(pulses) + " - Target: " + String(motor.target) + " - Voltage: " + String(voltage.q) + "/" + String(voltage.d));

    /*logSerial.print(currentSpeed);
    logSerial.print("\t");
    logSerial.println(motor.target);*/

    /*if (currentSpeed > 3 * vmax)
    {
      motor.controller = MotionControlType::velocity;
      motor.target = 0.0;
      continue;
    }*/

    if (launchDebug)
    {

      // logSerial.println("Event: launch debug");
      motor.controller = MotionControlType::torque;
      motor.target = 2.0;

      if (timeoutStart == 0)
      {
        timeoutStart = millis();
      }

      if (millis() - timeoutStart > 1000)
      {
        // logSerial.println("Event:end launch debug");

        launchDebug = false;
        timeoutStart = 0;

        motor.disable();

        motor.controller = MotionControlType::velocity;
        motor.target = 0.0;
      }
    }
    else
    {
      // TODO put in state machine
      if (state == 0 && pulses > pulseStart && currentSpeed > vmax && motor.target == 0 && timeoutStart == 0 && !closing)
      {
        // logSerial.println("Event: spinta");
        digitalWrite(ENABLE_PIN, HIGH);

        motor.enable();

        motor.controller = MotionControlType::torque;
        motor.target = -brakeVoltage(currentSpeed);
        state++;
      }

      if (state == 1 && pulses < pulseStop)
      {
        motor.target = -brakeVoltage(currentSpeed);
      }

      if (state == 1 && pulses > pulseStop /*&& motor.target == -tbrake*/ && timeoutStart == 0 && !closing)
      {
        // logSerial.println("Event: stop");
        motor.controller = MotionControlType::velocity;
        motor.target = vmin;
        state++;
      }

      if (state == 2 && pulses > pulseEnd && motor.controller == MotionControlType::velocity && timeoutStart == 0 && !closing)
      {
        // logSerial.println("Event: stop torque");
        motor.controller = MotionControlType::torque;
        motor.target = tend;
        state++;
      }

      if (state == 3 && timeoutStart == 0 && pulses > pulseEnd && motor.controller == MotionControlType::torque && (currentSpeed == 0 || pulses > RAIL_END_PULSES) && !closing)
      {
        // logSerial.println("Event: End");
        motor.target = 0;
        timeoutStart = millis();
        state++;
      }

      if (state == 4 && timeoutStart != 0 && millis() - timeoutStart > 5000 && !closing)
      {
        // logSerial.println("Event: close");
        motor.controller = MotionControlType::velocity;
        motor.target = -vmin;
        // timeoutStart = 0;
        closing = true;
        state++;
      }

      if (state == 5 && closing && (millis() - timeoutStart + 5000 > 500) && (pulses < pulseStart) && motor.controller == MotionControlType::velocity)
      {
        // logSerial.println("Event: close torque");

        motor.controller = MotionControlType::torque;
        motor.target = -tend / 2;
        state++;
      }

      if (state == 6 && closing && motor.controller == MotionControlType::torque && (currentSpeed == 0 /*|| pulses < RAIL_END_PULSES * 0.05*/))
      {
        // logSerial.println("Event: back to start");
        motor.target = 0;
        closing = false;
        state = 0;
        timeoutStart = 0;

        motor.disable();
        digitalWrite(ENABLE_PIN, LOW);
        // motor.sensor_offset=currentAngle; TODO
      }

      // if (state == 0 && pulses > pulseStart && currentSpeed > vmax && motor.target == 0 && timeoutStart == 0 && !closing)
    }

  }
}

void TaskSerial(void *pvParameters) // task comunicazione con seriale
{

  int lastSent = 0;
  while (1)
  {
    if (millis() - lastSent > 250)
    {
      serializeJson(data, logSerial);
      logSerial.println();
      lastSent = millis();
    }

    if (logSerial.available())
    {
      int tmpTarget = UNDEFINED_VALUE;
      float tmpvmax = UNDEFINED_VALUE;
      float tmpvmin = UNDEFINED_VALUE;
      float tmprampDuration = UNDEFINED_VALUE;
      int tmppulseStart = UNDEFINED_VALUE;
      int tmppulseStop = UNDEFINED_VALUE;
      int tmppulseEnd = UNDEFINED_VALUE;
      float tmptend = UNDEFINED_VALUE;

      String command = logSerial.readStringUntil('\n');

      // check if string contains stop
      if (command.indexOf("stop") >= 0)
      {
        motor.disable();
        logSerial.println("Stop command received");
      }

      // check if string contains restart
      if (command.indexOf("enable") >= 0)
      {
        motor.enable();
        logSerial.println("Restart command received");
      }

      if (command.indexOf("reset") >= 0)
      {
        launchDebug = false;
        timeoutStart = 0;

        motor.disable();

        motor.controller = MotionControlType::velocity;
        motor.target = 0.0;
        logSerial.println("Reset command received");
      }

      // check if string contains Set
      if (command.indexOf("Set") < 0)
      {
        continue;
      }

      sscanf(command.c_str(), "Set;%d;%f;%f;%f;%d;%d;%d;%f", &tmpTarget, &tmpvmax, &tmpvmin, &tmprampDuration, &tmppulseStart, &tmppulseStop, &tmppulseEnd, &tmptend);

      // TODO check input
      if (tmpTarget != UNDEFINED_VALUE)
      {
        motor.target = tmpTarget;
      }

      if (tmpvmax != UNDEFINED_VALUE)
      {
        vmax = tmpvmax;
      }

      if (tmpvmin != UNDEFINED_VALUE)
      {
        vmin = tmpvmin;
      }

      if (tmprampDuration != UNDEFINED_VALUE)
      {
        rampDuration = tmprampDuration;
      }

      if (tmppulseStart != UNDEFINED_VALUE)
      {
        pulseStart = tmppulseStart;
      }

      if (tmppulseStop != UNDEFINED_VALUE)
      {
        pulseStop = tmppulseStop;
      }

      if (tmppulseEnd != UNDEFINED_VALUE)
      {
        pulseEnd = tmppulseEnd;
      }

      if (tmptend != UNDEFINED_VALUE)
      {
        tend = tmptend;
      }
    }

    delay(10);
  }
}

void loop()
{
  motor.loopFOC();
  motor.move();
  // FOC algorithm function

  // motor.move();

  // currentAngle = sensor.getAngle();
  // currentSpeed = sensor.getVelocity();
}