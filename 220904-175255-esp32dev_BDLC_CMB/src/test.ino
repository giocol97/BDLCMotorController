#include "headers.h"

#include <SimpleFOC.h>
#include <WiFi.h>
#include <Preferences.h>

Preferences preferences;

// DynamicJsonDocument data(1024);

// valori JSON
long logTime = 0;
int logPulses = 0;
float logSpeed = 0;
float logVoltage = 0;
float logTarget = 0;
int logControl = 0;
int logState = 0;

HardwareSerial logSerial = Serial1;

// #define COMMANDER_ENABLED

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
// LowsideCurrentSense current_sense = LowsideCurrentSense(SHUNT_RESISTOR, CURRENT_SENSING_GAIN, I_U, I_V, I_W);

TaskHandle_t TaskHandleSpeed;
TaskHandle_t TaskHandleData;
TaskHandle_t TaskHandleWatchdog;

TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskControlHandle;
TaskHandle_t TaskHandleSerial;

void sensorA() { sensor.handleA(); }
void sensorB() { sensor.handleB(); }
void sensorC() { sensor.handleC(); }

// state variables

#define UNDEFINED_VALUE -123456
#define RAIL_LENGTH_DEBUG 2190
#define SPEED_ALMOST_ZERO 10

uint8_t currentSystemState = STATE_START;
uint8_t prevState = STATE_START;

float currentSpeed = UNDEFINED_VALUE;
float currentAngle = UNDEFINED_VALUE;
float lastSpeed = 0;

int movementDirection = DIRECTION_UNDEFINED;
bool currentPeakEventTriggered = false;
int standbyTimerStart = 0;

// sensor reading variables

float raw_speed[50] = {0};
float raw_angle[50] = {0};

int sensor_index = 0;

// web parameters TODO define defaults in header
float vmax = 100;                         // rad/s
float vmax_frenata = 400;                 // rad/s
float vmin_frenata = 200;                 // rad/s
float c_frenata = 3;                      // V*10
float vmin = 50;                          // rad/s
float v_tocco = 25;                       // rad/s
float rampDuration = UNDEFINED_VALUE;     // ms TODO use
int pulseStart = RAIL_LENGTH_DEBUG * 0.1; // pulses
int pulseStop = RAIL_LENGTH_DEBUG * 0.75; // pulses
int pulseEnd = RAIL_LENGTH_DEBUG * 0.9;   // pulses
float tend = 0.3;                         // V
float tbrake = 0.7;
long timeoutDuration = 10000;

int state = 0;

long endSetupMillis = 0;
int timeoutStart = 0;
bool closing = false;
bool launchDebug = false;

void initPins()
{
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(ENABLE_BAT, OUTPUT);
  digitalWrite(ENABLE_BAT, HIGH);

  /*pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);*/

 // analogReadResolution(10);
 // analogSetAttenuation(ADC_0db);

  pinMode(HALL_U, INPUT_PULLUP);
  pinMode(HALL_V, INPUT_PULLUP);
  pinMode(HALL_W, INPUT_PULLUP);

  pinMode(PWM_U, OUTPUT);
  pinMode(PWM_V, OUTPUT);
  pinMode(PWM_W, OUTPUT);
}

void initVariables()
{
  preferences.begin(CONFIG_NAMESPACE, false);

  vmax = preferences.getFloat("vmax", DEFAULT_VMAX);
  vmax_frenata = preferences.getFloat("vmax_frenata", DEFAULT_VMAX_FRENATA);
  vmin_frenata = preferences.getFloat("vmin_frenata", DEFAULT_VMIN_FRENATA);
  c_frenata = preferences.getFloat("c_frenata", DEFAULT_C_FRENATA);
  vmin = preferences.getFloat("vmin", DEFAULT_VMIN);
  v_tocco = preferences.getFloat("v_tocco", DEFAULT_VTOCCO);
  rampDuration = preferences.getFloat("rampDuration", DEFAULT_RAMP_DURATION);
  pulseStart = preferences.getInt("pulseStart", DEFAULT_PULSE_START);
  pulseStop = preferences.getInt("pulseStop", DEFAULT_PULSE_STOP);
  pulseEnd = preferences.getInt("pulseEnd", DEFAULT_PULSE_END);
  tend = preferences.getFloat("tend", DEFAULT_TEND);
  tbrake = preferences.getFloat("tbrake", DEFAULT_TBRAKE);
  timeoutDuration = preferences.getLong("timeoutDuration", DEFAULT_TIMEOUT_DURATION);

  preferences.end();
}

void setup()
{
  WiFi.mode(WIFI_OFF);

  // set CPU frequency to 80MHz TODO activate
  // setCpuFrequencyMhz(80);

  Serial.begin(LOG_BAUD);
  // logSerial.begin(LOG_BAUD);

  ledInit();
  ledMagenta();

  logSerial.begin(LOG_BAUD, SERIAL_8N1, LOG_RX, LOG_TX);
  logSerial.println("Starting new");

  // delay(500);

  //initVariables();

  initPins();

  // delay(50); // TODO rimuovere

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

  // current_sense.linkDriver(&driver);

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
  motor.LPF_velocity.Tf = 0.1;

  /*motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 1000;
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002f; // 1ms default
  motor.LPF_current_d.Tf = 0.002f; // 1ms default*/

  motor.voltage_limit = 16.8;
  // motor.current_limit = 4.3;
  // motor.velocity_limit = MAX_SPEED; // rad/s

  motor.foc_modulation = FOCModulationType::SinePWM;
  // initialize motor
  motor.init();

  // delay(100); // TODO remove

  logSerial.println("Motor pre FOC OK");

  // delay(100); // TODO remove

  motor.initFOC(0.0, Direction::CCW);
  logSerial.println("Motor OK");

  logSerial.println("Done configuring");

  motor.disable();
  digitalWrite(ENABLE_PIN, LOW);

  //  delay(500); // TODO remove

  currentSystemState = STATE_START;
  endSetupMillis = millis();

  xTaskCreatePinnedToCore(
      TaskControl,
      "TaskControl",
      5000,
      NULL,
      15,
      &TaskControlHandle,
      1);

  xTaskCreatePinnedToCore(
      TaskSensor,
      "TaskSensor",
      5000,
      NULL,
      10,
      &TaskSensorHandle,
      0);

  xTaskCreatePinnedToCore(
      TaskWatchdog,
      "TaskControl",
      5000,
      NULL,
      20,
      &TaskHandleWatchdog,
      0);

  xTaskCreatePinnedToCore(
      TaskSerial,
      "Taskserial",
      5000,
      NULL,
      5,
      &TaskHandleSerial,
      0);
}

void TaskSensor(void *pvParameters) // task raccolta dati/commander
{
  while (1)
  {
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

/*
float vmax = 100;                         // rad/s
float vmax_frenata = 500;                 // rad/s
float vmin_frenata = 150;                 // rad/s
float c_frenata = 3;                      // V*10
float vmin = 50;                          // rad/s
*/

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float brakeVoltage(float speed)
{
  return floatMap(speed, vmin_frenata, vmax_frenata, 0, c_frenata) / 10;
}

// determine from speed the direction of movement
int determineMovementDirection(int speed)
{
  if (speed > SPEED_ALMOST_ZERO)
  {
    return DIRECTION_POSITIVE;
  }
  else if (speed < -SPEED_ALMOST_ZERO)
  {
    return DIRECTION_NEGATIVE;
  }

  // speed==0
  return DIRECTION_UNDEFINED;
}

bool updateState(float speed, int millis, float target)
{
  switch (currentSystemState)
  {
  case STATE_START:
  {
    if (speed != UNDEFINED_VALUE)
    {
      currentSystemState = STATE_INCORSA;

      movementDirection = determineMovementDirection(speed);

      // print condition for state update
      logSerial.printf("STATE_START -> STATE_INCORSA speed(%.2f)\n", speed);
    }
    break;
  }
  case STATE_INCORSA:
  {
    if (abs(speed) <= SPEED_ALMOST_ZERO || currentPeakEventTriggered) // TODO currentPeakEventTriggered
    {
      currentPeakEventTriggered = false;
      currentSystemState = STATE_STANDBY;
      standbyTimerStart = millis;

      // print condition for state update
      logSerial.printf("STATE_INCORSA -> STATE_STANDBY speed(%.2f) == 0 || currentPeakEventTriggered(%d) == 1\n", speed, currentPeakEventTriggered);
    }
    break;
  }
  case STATE_STANDBY:
  {
    if (abs(speed) > v_tocco)
    {
      currentSystemState = STATE_INIZIO_RITORNO;
      standbyTimerStart = 0;

      movementDirection = determineMovementDirection(speed);

      // print condition for state update
      logSerial.printf("STATE_STANDBY -> STATE_INIZIO_RITORNO abs(speed)(%.2f) > v_tocco \n", abs(speed));
    }
    else if (standbyTimerStart!=0 && millis - standbyTimerStart > STANDBY_TIMEOUT)
    {

      // print condition for state update
      logSerial.printf("STATE_STANDBY -> SHUTDOWN millis(%d) - standbyTimerStart(%d) > %d\n", millis, standbyTimerStart, STANDBY_TIMEOUT);
      delay(100);

      digitalWrite(ENABLE_BAT, LOW);
    }
    break;
  }

  case STATE_INIZIO_RITORNO:
  {
    if (abs(motor.target) > tbrake && abs(speed) <= SPEED_ALMOST_ZERO) // probable obstacle or blocked by hand, stop and wait for new input or timeout
    {
      currentSystemState = STATE_STANDBY;
      // print condition for state update
      logSerial.printf("STATE_INIZIO_RITORNO -> STATE_STANDBY abs(motor.target)(%.2f) > tbrake(%.2f) && abs(speed)(%.2f) <= SPEED_ALMOST_ZERO\n", abs(motor.target), tbrake, abs(speed));
    }
    else if (abs(speed) > 60) // if close to v_return go to STATE_INCORSA to resume normal control
    {
      currentSystemState = STATE_INCORSA;
      // print condition for state update
      logSerial.printf("STATE_INIZIO_RITORNO -> STATE_INCORSA abs(speed)(%.2f) > 60\n", abs(speed));
    }
    break;
  }
  }
  return true;
}

void TaskControl(void *pvParameters) // task implementazione funzionalità
{

  while (1)
  {

    // int pulses = radiansToImpulses(currentAngle);
    float targetSpinta = 0;

    if (launchDebug) // usato per simulare lancio di anta su sistema con volano
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

      if (!updateState(currentSpeed, millis(), motor.target))
      {
        logSerial.println("Error, invalid state");
        motor.disable();
        continue;
      }

      // esecuzione comandi a motore

      switch (currentSystemState)
      {
      case STATE_START:
      {
        // do nothing
        break;
      }

      case STATE_INCORSA:
      {
        // targetSpinta = -movementDirection * brakeVoltage(abs(currentSpeed));
        // motor.controller = MotionControlType::torque;
        if (!motor.enabled)
        {
          digitalWrite(ENABLE_PIN, HIGH);
          motor.enable();
        }

        /*if (targetSpinta == 0)
        {
          digitalWrite(ENABLE_PIN, LOW);
        }
        else
        {
          digitalWrite(ENABLE_PIN, HIGH);
        }*/

        motor.controller = MotionControlType::velocity;

        targetSpinta = -movementDirection * vmin;

        motor.target = targetSpinta;
        break;
      }

      case STATE_STANDBY:
      {
        motor.target = 0;
        if (motor.enabled)
        {
          motor.disable();
          digitalWrite(ENABLE_PIN, LOW);
        }
        break;
      }

      case STATE_INIZIO_RITORNO:
      {
        motor.controller = MotionControlType::torque;
        if (!motor.enabled)
        {
          digitalWrite(ENABLE_PIN, HIGH);
          motor.enable();
        }

        motor.target += movementDirection * 0.25;

        break;
      }
      }
    }

    // update variables to be logged
    logTime = millis();
    logSpeed = currentSpeed;
    logVoltage = 0.0f;
    logTarget = motor.target;
    logControl = motor.controller;
    logState = currentSystemState;

    delay(50); // TODO?
  }
}

bool ledOn = false;

int lastUpdatedSerialTask = 0;

void TaskWatchdog(void *pvParameters) // task watchdog
{
  while (1)
  {
    delay(100);

    if (millis() - lastUpdatedSerialTask > 500)
    {
      Serial.println("Watchdog: Serial task not responding");
      ledBlue();
    }
  }
}

void TaskSerial(void *pvParameters) // task comunicazione con seriale
{

  int lastSent = 0;
  while (1)
  {

    lastUpdatedSerialTask = millis();

    if (millis() - lastSent > 100)
    {
      lastSent = millis();

      //{"time":6146,"pulses":-1467,"speed":-185.3429108,"voltage":0,"target":0.449999988,"control":0,"state":0}

      // serializeJson(data, logSerial);

      logSerial.print("{\"time\":");
      logSerial.print(logTime);
      logSerial.print(",\"pulses\":");
      logSerial.print(logPulses);
      logSerial.print(",\"speed\":");
      logSerial.print(logSpeed);
      logSerial.print(",\"voltage\":");
      logSerial.print(logVoltage);
      logSerial.print(",\"target\":");
      logSerial.print(logTarget);
      logSerial.print(",\"control\":");
      logSerial.print(logControl);
      logSerial.print(",\"state\":");
      logSerial.print(logState);
      logSerial.print(",\"enabled\":");
      logSerial.print(motor.enabled);
      logSerial.print(",\"phases(u_v_w)\":");
      logSerial.printf("%.2f_%.2f_%.2f", motor.Ua, motor.Ub, motor.Uc);
      logSerial.print("}");
      logSerial.println();

      // Serial.println(millis());

      if (ledOn)
      {
        ledRed();
      }
      else
      {
        ledGreen();
      }

      ledOn = !ledOn;
    }

    if (logSerial.available())
    {
      int tmpTarget = UNDEFINED_VALUE;
      float tmpvmax = UNDEFINED_VALUE;
      float tmpvmin = UNDEFINED_VALUE;
      float tmpvmaxfrenata = UNDEFINED_VALUE;
      float tmpvminfrenata = UNDEFINED_VALUE;
      float tmpcfrenata = UNDEFINED_VALUE;
      float tmpvtocco = UNDEFINED_VALUE;
      float tmprampDuration = UNDEFINED_VALUE;
      int tmppulseStart = UNDEFINED_VALUE;
      int tmppulseStop = UNDEFINED_VALUE;
      int tmppulseEnd = UNDEFINED_VALUE;
      float tmptend = UNDEFINED_VALUE;
      float tmptbrake = UNDEFINED_VALUE;
      int tmptimeoutDuration = UNDEFINED_VALUE;

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
        logSerial.println("enable command received");
      }

      if (command.indexOf("reset") >= 0)
      {
        ESP.restart();
      }

      logSerial.println("Command received: " + command);

      // check if string contains Set
      if (command.indexOf("Set") < 0)
      {
        continue;
      }

      sscanf(command.c_str(), "Set;%f;%f;%f;%d;%d;%d;%f;%f;%d;%f;%f;%f;%f", &tmpvmax, &tmpvmin, &tmprampDuration, &tmppulseStart, &tmppulseStop, &tmppulseEnd, &tmptend, &tmptbrake, &tmptimeoutDuration, &tmpvmaxfrenata, &tmpvminfrenata, &tmpcfrenata, &tmpvtocco);

      // TODO check input
      /*if (tmpTarget != UNDEFINED_VALUE)
      {
        motor.target = tmpTarget;
      }*/

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

      if (tmptbrake != UNDEFINED_VALUE)
      {
        tbrake = tmptbrake;
      }

      if (tmptimeoutDuration != UNDEFINED_VALUE)
      {
        timeoutDuration = tmptimeoutDuration;
      }

      if (tmpvmaxfrenata != UNDEFINED_VALUE)
      {
        vmax_frenata = tmpvmaxfrenata;
      }

      if (tmpvminfrenata != UNDEFINED_VALUE)
      {
        vmin_frenata = tmpvminfrenata;
      }

      if (tmpcfrenata != UNDEFINED_VALUE)
      {
        c_frenata = tmpcfrenata;
      }

      if (tmpvtocco != UNDEFINED_VALUE)
      {
        v_tocco = tmpvtocco;
      }

      logSerial.printf("Set parameters: vmax=%f, vmin=%f, rampDuration=%f, pulseStart=%d, pulseStop=%d, pulseEnd=%d, tend=%f, tbrake=%f, timeoutDuration=%d, vmax_frenata=%f, vmin_frenata=%f, c_frenata=%f, v_tocco=%f\n", vmax, vmin, rampDuration, pulseStart, pulseStop, pulseEnd, tend, tbrake, timeoutDuration, vmax_frenata, vmin_frenata, c_frenata, v_tocco);

      saveCurrentPreferences();
    }

    delay(10);
  }
}

void saveCurrentPreferences()
{
  Serial.println("Opening preferences");

  preferences.begin(CONFIG_NAMESPACE, false);
  preferences.putFloat("vmax", vmax);
  preferences.putFloat("vmin", vmin);
  preferences.putFloat("rampDuration", rampDuration);
  preferences.putInt("pulseStart", pulseStart);
  preferences.putInt("pulseStop", pulseStop);
  preferences.putInt("pulseEnd", pulseEnd);
  preferences.putFloat("tend", tend);
  preferences.putFloat("tbrake", tbrake);
  preferences.putLong("timeoutDuration", timeoutDuration);
  preferences.putFloat("vmax_frenata", vmax_frenata);
  preferences.putFloat("vmin_frenata", vmin_frenata);
  preferences.putFloat("c_frenata", c_frenata);
  preferences.putFloat("v_tocco", v_tocco);
  preferences.end();

  Serial.println("Preferences saved");
}

void loop() // loop only for SimpleFOC control
{
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();
}