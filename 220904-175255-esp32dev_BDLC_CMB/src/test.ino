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
#define RAIL_LENGTH_DEBUG 2190

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

int railLength = 0;

float v_configurazione = 10; // rad/s TODO confermare

int state = 0;

long endSetupMillis = 0;
int endStartStateMillis = 0;
int timeoutStart = 0;
bool closing = false;
bool launchDebug = false;

int countConfigurationLeft = 0;
int countConfigurationRight = 0;
int configurationDirection = 1; // 1=right, -1=left
int pulsesLeft = 0;
int pulsesRight = 0;
int timeFromConfigStart = 0;

bool configurationDone = false;

int timeoutRicarica = 0;
int timeoutFineRicarica = 0;

void initPins()
{
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(ENABLE_BAT, OUTPUT);
  digitalWrite(ENABLE_BAT, HIGH);

  /*pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);*/

  analogReadResolution(10);
  analogSetAttenuation(ADC_0db);

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

  railLength = preferences.getInt("railLength", RAIL_LENGTH_DEBUG);

  pulseStart = railLength * 0.1;
  pulseStop = railLength * 0.75;
  pulseEnd = railLength * 0.9;

  vmax = preferences.getFloat("vmax", DEFAULT_VMAX);
  vmax_frenata = preferences.getFloat("vmax_frenata", DEFAULT_VMAX_FRENATA);
  vmin_frenata = preferences.getFloat("vmin_frenata", DEFAULT_VMIN_FRENATA);
  c_frenata = preferences.getFloat("c_frenata", DEFAULT_C_FRENATA);
  vmin = preferences.getFloat("vmin", DEFAULT_VMIN);
  v_tocco = preferences.getFloat("v_tocco", DEFAULT_VTOCCO);
  rampDuration = preferences.getFloat("rampDuration", DEFAULT_RAMP_DURATION);
  /*pulseStart = preferences.getInt("pulseStart", DEFAULT_PULSE_START);
  pulseStop = preferences.getInt("pulseStop", DEFAULT_PULSE_STOP);
  pulseEnd = preferences.getInt("pulseEnd", DEFAULT_PULSE_END);*/
  tend = preferences.getFloat("tend", DEFAULT_TEND);
  tbrake = preferences.getFloat("tbrake", DEFAULT_TBRAKE);
  timeoutDuration = preferences.getLong("timeoutDuration", DEFAULT_TIMEOUT_DURATION);

  // print all variables
  logSerial.printf("railLength: %d - vmax: %.2f - vmax_frenata: %.2f - vmin_frenata: %.2f - c_frenata: %.2f - vmin: %.2f - v_tocco: %.2f - rampDuration: %.2f - pulseStart: %d - pulseStop: %d - pulseEnd: %d - tend: %.2f - tbrake: %.2f - timeoutDuration: %d\n", railLength, vmax, vmax_frenata, vmin_frenata, c_frenata, vmin, v_tocco, rampDuration, pulseStart, pulseStop, pulseEnd, tend, tbrake, timeoutDuration);

  preferences.end();
}

void setup()
{
  WiFi.mode(WIFI_OFF);

  Serial.begin(LOG_BAUD);
  // logSerial.begin(LOG_BAUD);

  ledInit();
  ledMagenta();

  logSerial.begin(LOG_BAUD, SERIAL_8N1, LOG_RX, LOG_TX);
  logSerial.println("Starting new");

  // delay(500);

  initVariables();

  initPins();

  delay(50);

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

  // print current sense pins
  // logSerial.printf("Current sense pins: %d %d %d \n", current_sense.pinA, current_sense.pinB, current_sense.pinC);

  /*// init current sense TODO disattivato
  if (current_sense.init())
    logSerial.println("Current sense init success!");
  else
  {
    logSerial.println("Current sense init failed!");
    return;
  }

  // link the motor to current sense
  motor.linkCurrentSense(&current_sense);*/

  /*current_sense.gain_a = -5.0f;
  current_sense.gain_b = -5.0f;
  current_sense.gain_c = -5.0f;*/

  // current_sense.skip_align = true;

  logSerial.println("Motor pre FOC OK");

  delay(100);

  motor.initFOC(0.0, Direction::CCW);
  logSerial.println("Motor OK");

  logSerial.println("Done configuring");

  motor.disable();
  digitalWrite(ENABLE_PIN, LOW);

  // delay(1000);

  // logSerial.printf("Current sense pins: %d %d %d \n", current_sense.pinA, current_sense.pinB, current_sense.pinC);
  delay(500);

  currentSystemState = STATE_START;
  endSetupMillis = millis();
  // motor.target = (300);

  // #ifndef COMMANDER_ENABLED
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
      Task1,
      "Task1",
      5000,
      NULL,
      15,
      &TaskHandle1,
      1);

  xTaskCreatePinnedToCore(
      Task0,
      "Task0",
      5000,
      NULL,
      10,
      &TaskHandle0,
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
  // #endif
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

bool updateState(int pulses, float speed, int millis)
{
  switch (currentSystemState)
  {
  case STATE_START:
    if (millis - endSetupMillis > 2000)
    {
      endStartStateMillis = millis;
      currentSystemState = STATE_FINESTART;
      logSerial.printf("STATE_START -> STATE_FINESTART millis(%d) - endSetupMillis(%d) > 2000\n", millis, endSetupMillis);
    }

    break;
  case STATE_FINESTART:
  {
    if (millis - endStartStateMillis > 500)
    {
      if (speed == 0)
      {
        sensor.electric_rotations = 0;
        currentSystemState = STATE_INACTIVE;

        // print condition for state update
        logSerial.printf("STATE_START -> STATE_INACTIVE millis(%d) - endSetupMillis(%d) > 2000 && speed(%.2f) == 0\n", millis, endSetupMillis, speed);
      }
      else if (speed != 0)
      {
        currentSystemState = STATE_RICARICA;
        logSerial.printf("STATE_START -> STATE_RICARICA speed(%.2f) != 0\n", speed);
      }
    }
  }
  break;
  case STATE_RICARICA:
  {
    if (speed == 0)
    {
      if (timeoutRicarica == 0)
      {
        timeoutRicarica == millis;
      }

      if (millis - timeoutRicarica > 3000)
      {
        timeoutFineRicarica = millis;
        currentSystemState = STATE_FINERICARICA;
        logSerial.printf("STATE_RICARICA -> STATE_FINERICARICA speed(%.2f) == 0 && millis(%d) - timeoutRicarica(%d) > 3000\n", speed, millis, timeoutRicarica);
      }
    }
    else
    {
      timeoutRicarica = 0;
    }
  }
  break;
  case STATE_FINERICARICA:
  {
    if (speed == 0 && millis - timeoutFineRicarica > 1000)
    {
      currentSystemState = STATE_INACTIVE;
      sensor.electric_rotations = 0;
      logSerial.printf("STATE_FINERICARICA -> STATE_INACTIVE speed(%.2f) == 0\n", speed);
    }
  }
  break;
  case STATE_CONFIGURAZIONE:
  {
    pulsesLeft = min(pulsesLeft, pulses);
    pulsesRight = max(pulsesRight, pulses);

    if (speed == 0 && millis - timeFromConfigStart > 1000) // TODO filtro su velocità?
    {
      if (countConfigurationLeft == 1 && countConfigurationRight == 1) // configurati entrambi i lati, fine configurazione
      {
        // save rail length configuration to Preferences
        preferences.begin(CONFIG_NAMESPACE, false);
        preferences.putInt("railLength", pulsesRight - pulsesLeft);

        currentSystemState = STATE_INACTIVE;

        logSerial.printf("railLength: %d from pulsesLeft(%d) pulsesRight(%d)\n", pulsesRight - pulsesLeft, pulsesLeft, pulsesRight);

        logSerial.printf("STATE_CONFIGURAZIONE -> STATE_INACTIVE countConfigurationLeft(%d) == 2 && countConfigurationRight(%d) == 2\n", countConfigurationLeft, countConfigurationRight);
      }
      else if (configurationDirection == 1)
      {
        configurationDirection = -1;
        countConfigurationRight++;
        timeFromConfigStart = millis;
      }
      else if (configurationDirection == -1)
      {
        configurationDirection = 1;
        countConfigurationLeft++;
        timeFromConfigStart = millis;
      }
    }
  }
  break;
  case STATE_INACTIVE:
    if (!configurationDone || digitalRead(BUTTON_PIN))
    {
      configurationDone = true;
      timeFromConfigStart = millis;
      currentSystemState = STATE_CONFIGURAZIONE;

      logSerial.printf("STATE_INACTIVE -> STATE_CONFIGURAZIONE digitalRead(BUTTON_PIN)(%d)\n", digitalRead(BUTTON_PIN));
    }
    else if (pulses > pulseStart && speed > vmax)
    {
      currentSystemState = STATE_SPINTA;

      // print condition for state update
      logSerial.printf("STATE_INACTIVE -> STATE_SPINTA pulses(%d) > pulseStart(%d) && speed(%.2f) > vmax(%.2f)\n", pulses, pulseStart, speed, vmax);
    }
    else if (pulses > pulseEnd && speed < vmin)
    {
      currentSystemState = STATE_FINECORSA;

      // print condition for state update
      logSerial.printf("STATE_INACTIVE -> STATE_FINECORSA pulses(%d) > pulseEnd(%d) && speed(%.2f) < vmin(%.2f)\n", pulses, pulseEnd, speed, vmin);
    }
    break;
  case STATE_SPINTA:
    if (pulses > pulseStop || speed < vmin)
    {
      currentSystemState = STATE_FRENATA;

      // print condition for state update
      logSerial.printf("STATE_SPINTA -> STATE_FRENATA pulses(%d) > pulseStop(%d) || speed(%.2f) < vmin(%.2f)\n", pulses, pulseStop, speed, vmin);
    }
    break;
  case STATE_FRENATA:
    if (pulses > pulseEnd)
    {
      currentSystemState = STATE_QUASIFINECORSA;

      // print condition for state update
      logSerial.printf("STATE_FRENATA -> STATE_QUASIFINECORSA pulses(%d) > pulseEnd(%d)\n", pulses, pulseEnd);
    }
    break;
  case STATE_QUASIFINECORSA:
    if (speed == 0)
    { // TODO mettere speed "circa" 0?
      currentSystemState = STATE_FINECORSA;

      // print condition for state update
      logSerial.printf("STATE_QUASIFINECORSA -> STATE_FINECORSA speed(%.2f) == 0\n", speed);
    }
    break;
  case STATE_FINECORSA:
    if (millis - timeoutStart > timeoutDuration || speed < -v_tocco)
    {
      timeoutStart = 0;
      currentSystemState = STATE_INIZIO_RITORNO;

      // print condition for state update
      logSerial.printf("STATE_FINECORSA -> STATE_INIZIO_RITORNO millis(%d) - timeoutStart(%d) > timeoutDuration(%d) || speed(%.2f) < -v_tocco(%.2f)\n", millis, timeoutStart, timeoutDuration, speed, -v_tocco);
    }
    break;
  case STATE_INIZIO_RITORNO:
    if (speed < -vmin - 10 /*|| pulses > pulseEnd*/)
    {
      currentSystemState = STATE_RITORNO_VEL;

      // print condition for state update
      logSerial.printf("STATE_INIZIO_RITORNO -> STATE_RITORNO_VEL speed(%.2f) < -vmin(%.2f) - 10\n", speed, -vmin);
    }
    break;
  case STATE_RITORNO_VEL:
    if (pulses < pulseStart)
    {
      currentSystemState = STATE_RITORNO_TOR;

      // print condition for state update
      logSerial.printf("STATE_RITORNO_VEL -> STATE_RITORNO_TOR pulses(%d) < pulseStart(%d)\n", pulses, pulseStart);
    }
    break;
  case STATE_RITORNO_TOR:
    if (speed == 0)
    {
      sensor.electric_rotations = 0;
      currentSystemState = STATE_INACTIVE;

      // print condition for state update
      logSerial.printf("STATE_RITORNO_TOR -> STATE_INACTIVE speed(%.2f) == 0\n", speed);
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
    float targetSpinta = 0;

    // DQVoltage_s voltage = motor.voltage;

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

      if (!updateState(pulses, currentSpeed, millis()))
      {
        logSerial.println("Error, invalid state");
        motor.disable();
        continue;
      }

      // esecuzione comandi a motore

      switch (currentSystemState)
      {
      case STATE_START:
        motor.controller = MotionControlType::torque;
        if (!motor.enabled)
        {
          digitalWrite(ENABLE_PIN, HIGH);
          motor.enable();
        }

        // motor.controller = MotionControlType::torque;
        motor.target = 0.45;
        break;
      case STATE_FINESTART:
      {
        motor.target = 0;
        if (motor.enabled)
        {
          motor.disable();
          digitalWrite(ENABLE_PIN, LOW);
        }
        break;
      }
      break;
      case STATE_CONFIGURAZIONE:
      {
        motor.controller = MotionControlType::torque;
        if (!motor.enabled)
        {
          digitalWrite(ENABLE_PIN, HIGH);
          motor.enable();
        }

        if (abs(currentSpeed) < 15)
        {
          motor.target = 1 * configurationDirection;
        }

        if (abs(currentSpeed) > 35)
        {
          motor.target = 0;
        }
      }
      break;
      case STATE_RICARICA:
      {
        motor.target = 0;
        if (motor.enabled)
        {
          motor.disable();
          digitalWrite(ENABLE_PIN, LOW);
        }
        break;
      }
      break;
      case STATE_FINERICARICA:
      {
        motor.controller = MotionControlType::torque;
        if (!motor.enabled)
        {
          digitalWrite(ENABLE_PIN, HIGH);
          motor.enable();
        }

        // motor.controller = MotionControlType::torque;
        motor.target = 0.75;
      }
      break;
      case STATE_INACTIVE:
        // TODO reset inizio corsa
        motor.target = 0;
        if (motor.enabled)
        {
          motor.disable();
          digitalWrite(ENABLE_PIN, LOW);
        }
        break;
      case STATE_SPINTA:
        targetSpinta = brakeVoltage(currentSpeed);
        motor.controller = MotionControlType::torque;
        if (!motor.enabled)
        {
          digitalWrite(ENABLE_PIN, HIGH);
          motor.enable();
        }

        if (targetSpinta == 0)
        {
          digitalWrite(ENABLE_PIN, LOW);
        }
        else
        {
          digitalWrite(ENABLE_PIN, HIGH);
        }

        motor.target = targetSpinta;
        break;
      case STATE_FRENATA:
        digitalWrite(ENABLE_PIN, HIGH);
        motor.controller = MotionControlType::velocity;
        motor.target = -vmin;
        break;
      case STATE_QUASIFINECORSA:
        motor.controller = MotionControlType::torque;
        motor.target = -tend;
        break;
      case STATE_FINECORSA:
        // TODO reset fine corsa
        motor.target = 0;

        if (motor.enabled)
        {
          motor.disable();
          digitalWrite(ENABLE_PIN, LOW);
        }

        if (timeoutStart == 0)
        {
          timeoutStart = millis();
        }
        break;
      case STATE_INIZIO_RITORNO:
        motor.controller = MotionControlType::torque;
        if (!motor.enabled)
        {
          digitalWrite(ENABLE_PIN, HIGH);
          motor.enable();
        }

        motor.target += 0.05;

        break;
      case STATE_RITORNO_VEL:
        motor.controller = MotionControlType::velocity;
        motor.target = vmin + 20;
        break;
      case STATE_RITORNO_TOR:
        motor.controller = MotionControlType::torque;
        motor.target = tend * 1.2;
        break;
      }
    }

    /* data["time"] = millis();
     // data["angle"] = currentAngle;
     data["pulses"] = pulses;
     data["speed"] = currentSpeed;
     data["voltage"] = 0.0f;
     data["target"] = motor.target;
     data["control"] = motor.controller;
     data["state"] = currentSystemState;*/

    logTime = millis();
    logPulses = pulses;
    logSpeed = currentSpeed;
    logVoltage = 0.0f;
    logTarget = motor.target;
    logControl = motor.controller;
    logState = currentSystemState;
    // serializeJson(data, logSerial); su altro task
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
  bool sendDataEnabled = false;

  while (1)
  {

    lastUpdatedSerialTask = millis();

    if (millis() - lastSent > 100 && sendDataEnabled)
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

      if (command.indexOf("TYPE") >= 0)
      {
        logSerial.println("{\"TYPE\":\"SALICE\"}");
        continue;
      }

      if (command.indexOf("sendData0") >= 0)
      {
        sendDataEnabled = false;
        logSerial.println("sendData Off");
      }

      if (command.indexOf("sendData1") >= 0)
      {
        sendDataEnabled = true;
        logSerial.println("sendData On");
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

      if (command.indexOf("Get;") >= 0)
      {        
        logSerial.printf("Get;%f;%f;%f;%d;%d;%d;%f;%f;%d;%f;%f;%f;%f;\n", vmax, vmin, rampDuration, pulseStart, pulseStop, pulseEnd, tend, tbrake, timeoutDuration, vmax_frenata, vmin_frenata, c_frenata, v_tocco); 
        continue;
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

void loop()
{
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();
}