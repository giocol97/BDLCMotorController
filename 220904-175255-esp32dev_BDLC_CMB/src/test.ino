#include <SimpleFOC.h>

// Hall sensor instance
// HallSensor(int hallA, int hallB , int hallC , int pp)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(17, 5, 18, 8); // TODO

//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC)
BLDCDriver3PWM driver = BLDCDriver3PWM(27, 14, 12); // TODO

// current sense?

//  BLDCMotor( pole_pairs , ( phase_resistance, KV_rating  optional) )
BLDCMotor motor = BLDCMotor(8, 0.33, 220); //, 220); // TODO

// Commander command = Commander(Serial);

// void doMotor(char *cmd) { command.motor(&motor, cmd); }

LowsideCurrentSense current_sense = LowsideCurrentSense(0.1, 10, 32, 33, 25);

TaskHandle_t TaskHandleSpeed;
TaskHandle_t TaskHandleData;

float currentAngle = 0;
float currentSpeed = 0;

void sensorA() { sensor.handleA(); }
void sensorB() { sensor.handleB(); }
void sensorC() { sensor.handleC(); }

void initPins()
{
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  pinMode(32, INPUT);
  pinMode(33, INPUT);
  pinMode(25, INPUT);

  analogReadResolution(10);
  analogSetAttenuation(ADC_0db);
  /*
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);

    pinMode(33, OUTPUT);
    digitalWrite(33, HIGH);

    pinMode(36, OUTPUT);
    digitalWrite(36, HIGH);

    pinMode(37, OUTPUT);
    digitalWrite(37, HIGH);*/
}

void setup()
{

  Serial.begin(115200);
  Serial.println("Starting new");

  delay(1000);

  initPins();

  // initialize sensor hardware
  Serial.println("Initializing sensors");
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(sensorA, sensorB, sensorC);
  Serial.println("Sensors OK");

  // pwm frequency to be used [Hz]
  Serial.println("Initializing driver");
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 24; // 14.4;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 16.8;
  // driver init
  driver.init();
  Serial.println("Driver OK");

  current_sense.linkDriver(&driver);

  // Serial.println("Initializing commander");
  // command.add('M', doMotor, "motor");

  // use monitoring with the BLDCMotor
  // monitoring port
  Serial.println("Initializing motor");
  motor.useMonitoring(Serial);

  // init sensor
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // init driver
  // link the motor to the driver
  motor.linkDriver(&driver);

  // link the motor to current sense
  motor.linkCurrentSense(&current_sense);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = 24;//16.8;
  motor.current_limit = 7.1;
  motor.velocity_limit = 500;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // initialize motor
  motor.init();

  // init current sense
  current_sense.init();

  Serial.println("Motor pre FOC OK");

  delay(100);

  motor.initFOC();
  Serial.println("Motor OK");

  Serial.println("Done configuring");

  // motor.target = (300);

  xTaskCreatePinnedToCore(
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
      1);
}

void TaskControlSpeed(void *pvParameters)
{
  float target_speeds[6] = {-60.0, 60.0, 100.0, -100.0, 150.0, -150.0};

  while (1)
  {
    for (int i = 0; i < 6; i++)
    {
      Serial.print("Setting speed to ");
      Serial.print(target_speeds[i]);
      Serial.println(" rad/s");
      motor.target = (target_speeds[i]);
      vTaskDelay(3000);
    }
  }
  // motor.target=(3.0 * -3.14);
}

void TaskPrintData(void *pvParameters)
{

  int curA, curB, curC = 0;

  while (1)
  {
    //Serial.printf("Motor - Speed: %f rad/s, Angle: %f rad\n" /*, Current: %f A, Voltage: %f V, Power: %f W"*/,
    //              currentSpeed, currentAngle /*, motor.getCurrent(), motor.getVoltage(), motor.getPower()*/);


    float error= (-currentSpeed-motor.target)/motor.target;
    Serial.printf("Error: %f\n", error*100);

    /*for (int i = 0; i < 50; i++)
    {
      curA += analogRead(32);
      curB += analogRead(33);
      curC += analogRead(25);
      delay(1);
    }

    curA /= 50;
    curB /= 50;
    curC /= 50;

    Serial.printf("Currents - A: %d, B: %d, C: %d, sum: %d\n", curA, curB, curC, curA + curB + curC);*/

    delay(1000);
  }
}

void loop()
{
  // FOC algorithm function
  motor.loopFOC();
  motor.move();
  // motor.monitor();
  // command.run();

  // motor.move();

  currentAngle = sensor.getAngle();
  currentSpeed = sensor.getVelocity();
}