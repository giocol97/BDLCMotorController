#include "Freenove_WS2812_Lib_for_ESP32.h"

#include <SimpleFOC.h>
#include "current_sense/hardware_specific/esp32/esp32_adc_driver.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
void feedTheDog(){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  // feed dog 1
  //TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  //TIMERG1.wdt_feed=1;                       // feed dog
  //TIMERG1.wdt_wprotect=0;                   // write protect
}


#define LEDS_COUNT  1
#define LEDS_PIN  23
#define CHANNEL   1

#define EnBat 22
#define EnMot 21
#define Puls2 19
#define PWMu 27
#define PWMv 14
#define PWMw 12
#define H1 17
#define H2 5
#define H3 18
//#define DRVfault 23
//#define LED 23
#define BRK 26
//#define BRKfault 4
#define ADC_b 34
#define ADC_u 32
#define ADC_v 33
#define ADC_w 13

#include <SimpleFOC.h>


uint32_t gPlotTime_ms=100;
int8_t gPlotType=-1;

// Hall sensor instance
// HallSensor(int hallA, int hallB , int hallC , int pp)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(H1, H2, H3, 8);

// Interrupt routine initialization
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

TaskHandle_t TaskHandleData;

// define BLDC motor
BLDCMotor motor = BLDCMotor(8);
// define BLDC driver
BLDCDriver3PWM driver = BLDCDriver3PWM(PWMu, PWMv, PWMw, EnMot);

//LowsideCurrentSense current_sense = LowsideCurrentSense(0.1, 10, ADC_u, ADC_v, ADC_w);

float target_val = 0;
// commander interface
Commander command = Commander(Serial1);
void onTarget(char* cmd){ command.scalar(&target_val, cmd); }
void onReset(char* cmd){ ESP.restart(); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void onPlotType(char* cmd){ gPlotType=atoi(cmd); }
void onPlotTime(char* cmd){ gPlotTime_ms=atoi(cmd); }

void initPins()
{
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  pinMode(ADC_u, INPUT);
  pinMode(ADC_v, INPUT);
  pinMode(ADC_w, INPUT);

  analogReadResolution(10);
  analogSetAttenuation(ADC_0db);
  
    //pinMode(3, OUTPUT);
    //digitalWrite(3, HIGH);

    //pinMode(33, OUTPUT);
    ///digitalWrite(33, HIGH);

    //pinMode(36, OUTPUT);
    //digitalWrite(36, HIGH);

    pinMode(37, OUTPUT);
    //digitalWrite(37, HIGH);
}


void setup() {

  initPins();
  
  // monitoring port
  Serial.begin(250000);

  Serial1.begin(115200, SERIAL_8N1, 15, 4);

  SimpleFOCDebug::enable(&Serial);

  // check if you need internal pullups
  //sensor.pullup = Pullup::USE_EXTERN;
  sensor.pullup = Pullup::USE_INTERN;
  
  
  // initialize sensor hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);


  Serial.println("Sensor ready");
  //_delay(1000);



 // pwm frequency to be used [Hz]
  Serial.println("Initializing driver");
  driver.pwm_frequency = 40000;
  // power supply voltage [V]
  driver.voltage_power_supply = 24; // 14.4;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 24; //16.8;
  // driver init
  driver.init();

  //current_sense.linkDriver(&driver);


  motor.linkSensor(&sensor);
  // link the motor to the driver
  motor.linkDriver(&driver);

  //motor.linkCurrentSense(&current_sense);

  //motor.controller = MotionControlType::torque;
  //motor.voltage_limit = 24;//16.8;
  //motor.current_limit = 1;
  //motor.velocity_limit = 60;
  motor.phase_resistance = 1.42; // Ohms - default not set
  // motor KV rating [rpm/V]
  motor.KV_rating = 384.14; // rpm/volt - default not set

  // choose FOC modulation
  // FOCModulationType::SinePWM; (default)
  // FOCModulationType::SpaceVectorPWM;
  // FOCModulationType::Trapezoid_120;
  // FOCModulationType::Trapezoid_150;
  motor.foc_modulation = FOCModulationType::SinePWM;  

  motor.init();

  //current_sense.init();

  motor.initFOC();


  xTaskCreatePinnedToCore(
      TaskPrintData,
      "TaskLoop",
      5000,
      NULL,
      5,
      &TaskHandleData,
      0);
  

  // add target command T
  command.add('r', onTarget, "target");
  command.add('R', onReset, "reset board");
  command.add('M', doMotor, "motor");
  command.add('t', onPlotTime, "plot time");
  command.add('d', onPlotType, "plot type");
  motor.useMonitoring(Serial1);

  
}



double gCompLoop_Hz,gCompThread1_Hz;

#define CS_N 5
uint16_t CsI;
uint16_t CS_U[CS_N];
uint16_t CS_V[CS_N];
uint16_t CS_W[CS_N];



void TaskPrintData(void *pvParameters)
{
  uint32_t pt=0,r=0,h=0;

  while (1)
  {
    r=micros();
    if(r<pt) pt=r;
    
    //FOC Studio
    motor.monitor();

   // user communication
    command.run();


    //serial plot
    if((r-pt)>(gPlotTime_ms*1000)){
      if(gPlotType==0){
        Serial.printf("%f\t%f\t%f\n",gCompLoop_Hz,gCompLoop_Hz,gCompThread1_Hz);
      }
      else if(gPlotType==1 || gPlotType==2 || gPlotType==3 || gPlotType==4|| gPlotType==5|| gPlotType==6){
       float cu=0.0,cv=0.0,cw=0.0;
        for(int i=0;i<CS_N;i++){
          cu+=CS_U[i];
          cv+=CS_V[i];
          cw+=CS_W[i];
        }
        cu/=CS_N;
        cv/=CS_N;
        cw/=CS_N;
        
        //plot one
        if(gPlotType==1) Serial.printf("%f\t%f\t%f\n",cu,cu,cu);
        if(gPlotType==2) Serial.printf("%f\t%f\t%f\n",cv,cv,cv);
        if(gPlotType==3) Serial.printf("%f\t%f\t%f\n",cw,cw,cw);

        //plot all
        if(gPlotType==6) Serial.printf("%f\t%f\t%f\n",cu,cv,cw);
     
      }
      
      pt=r;
    }


    //read current sense
    CS_U[CsI]=analogRead(ADC_u);
    CS_V[CsI]=analogRead(ADC_v);
    CS_W[CsI]=analogRead(ADC_w);
    CsI=(CsI+1)%CS_N;

    
    feedTheDog();


    gCompThread1_Hz=1.0/((double)r/1E6-(double)h/1E6);
    h=r;
    
  }
}


uint32_t rl=0,hl=0;
void loop() {
  rl=micros();
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  sensor.update();
 
  // iterative foc function 
  motor.loopFOC();

  // iterative function setting and calculating the velocity loop
  // this function can be run at much lower frequency than loopFOC function
  motor.move(target_val);
  

  gCompLoop_Hz=1.0/((double)rl/1E6-(double)hl/1E6);
  hl=rl;

}
