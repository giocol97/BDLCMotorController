#include "Freenove_WS2812_Lib_for_ESP32.h"
#include "driver/ledc.h"
#include "xtensa/core-macros.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

void feedTheDog(){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  // feed dog 1 [LOOP runs on core ]
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
#define PWMu 12
#define PWMv 14
#define PWMw 27
#define H1 17
#define H2 5
#define H3 18
#define BRK 26
#define ADC_batt 34
#define ADC_brake 35
#define ADC_u 32
#define ADC_v 33
#define ADC_w 25

#define N_POLE_PAIRS 8 // motor pole pairs

///////////////////////////////////////////////////
// TASKS DEFS
///////////////////////////////////////////////////
TaskHandle_t TaskHandleSerialDbg;
void TaskSerialDbg(void *pvParameters);


///////////////////////////////////////////////////
// GLOBAL VARIABLES
///////////////////////////////////////////////////
double CLOCK_PERIOD_S; // board clock period (1/clock_freq)

double gRpmSp; //velocity setpoint
double gCompVelRpm; // computed motor velocity 

const int gAdcResolution = 10;  



double gPahseAng=0; // current phase loop 
double gPowerSp=0.0; //power % setpoint
double gDeg_s; //computed degs/s from rpm
double gCompLoop_Hz; //computed main loop freq
double gCompThread1_Hz; //computed thread loop freq
bool gHallRephaseEn;




int gWfLookup=0;

#define WAVEFORM_LOOKUP_TABLE_SIZE 360

static float *SINE_LOOKUP_TABLE;
static float *SINE3RD_LOOKUP_TABLE;
static float *SADDLE_LOOKUP_TABLE;
static float *SADDLE1_LOOKUP_TABLE;

void generateWaveformLookupTables(){
  float r=360.0/WAVEFORM_LOOKUP_TABLE_SIZE;
  
  SINE_LOOKUP_TABLE = (float*) malloc(WAVEFORM_LOOKUP_TABLE_SIZE*sizeof(float));
  SINE3RD_LOOKUP_TABLE = (float*) malloc(WAVEFORM_LOOKUP_TABLE_SIZE*sizeof(float));
  SADDLE_LOOKUP_TABLE = (float*) malloc(WAVEFORM_LOOKUP_TABLE_SIZE*sizeof(float));
  SADDLE1_LOOKUP_TABLE = (float*) malloc(WAVEFORM_LOOKUP_TABLE_SIZE*sizeof(float));
  //Serial1.printf("sin\tsin3rd\tsaddle\n");
  for(int i=0;i<WAVEFORM_LOOKUP_TABLE_SIZE;i++){
    SINE_LOOKUP_TABLE[i]=sin(r*i*M_PI/180.0);
    SINE_LOOKUP_TABLE[i]=(SINE_LOOKUP_TABLE[i]+1.0)/2.0;
    
    SINE3RD_LOOKUP_TABLE[i]=sin(r*i*M_PI/180.0) + 1.0/6.0*sin(r*i*3*M_PI/180.0);
    SINE3RD_LOOKUP_TABLE[i]=(SINE3RD_LOOKUP_TABLE[i]+1.0)/2.0;
    
    if((r*i)>=0 && (r*i)<120.0) SADDLE_LOOKUP_TABLE[i]=sin(r*i*M_PI/180.0);
    else if((r*i)>=120 && (r*i)<240){
      SADDLE_LOOKUP_TABLE[i]=sin((r*(i+300/r))*M_PI/180.0);
    }
    else SADDLE_LOOKUP_TABLE[i]=0;
    
    if((r*i)>=0 && (r*i)<90.0) SADDLE1_LOOKUP_TABLE[i]=sin(r*i*3.0/2.0*M_PI/180.0);
    else if((r*i)>=90 && (r*i)<270){
      SADDLE1_LOOKUP_TABLE[i]=-sin((r*(i+300/r))*3.0/2.0*M_PI/180.0);
    }
    else if((r*i)>=270 && (r*i)<360.0) SADDLE1_LOOKUP_TABLE[i]=-sin(r*i*3.0/2.0*M_PI/180.0);
    else SADDLE1_LOOKUP_TABLE[i]=0;
    SADDLE1_LOOKUP_TABLE[i]=(SADDLE1_LOOKUP_TABLE[i]+1.0)/2.0;
     
    
    // DBG: print waveform on Serial1
    //Serial1.printf("%f\t%f\t%f\n",SINE_LOOKUP_TABLE[i],SINE3RD_LOOKUP_TABLE[i],SADDLE_LOOKUP_TABLE[i]);
    //Serial1.printf("%f\n",SADDLE_LOOKUP_TABLE[i]*100);
  }
  
  // DBG: print pahses waveforms on Serial1
  //Serial1.printf("U\tV\tW\n");
  //for(int i=0;i<WAVEFORM_LOOKUP_TABLE_SIZE;i++){
    //Serial1.printf("%f\t%f\t%f\n",SINE_LOOKUP_TABLE[i]*100,SINE_LOOKUP_TABLE[(i+(int)(120/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100,SINE_LOOKUP_TABLE[(i+(int)(240/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100);
    //Serial1.printf("%f\t%f\t%f\n",SINE3RD_LOOKUP_TABLE[i]*100,SINE3RD_LOOKUP_TABLE[(i+(int)(120/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100,SINE3RD_LOOKUP_TABLE[(i+(int)(240/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100);
    //Serial1.printf("%f\t%f\t%f\n",SADDLE_LOOKUP_TABLE[i]*100,SADDLE_LOOKUP_TABLE[(i+(int)(120/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100,SADDLE_LOOKUP_TABLE[(i+(int)(240/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100);
    //Serial1.printf("%f\t%f\t%f\n",SADDLE1_LOOKUP_TABLE[i]*100,SADDLE1_LOOKUP_TABLE[(i+(int)(120/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100,SADDLE1_LOOKUP_TABLE[(i+(int)(240/r))%WAVEFORM_LOOKUP_TABLE_SIZE]*100);
  //}

  
}

// get waveform value at given angle in DEG (global variable 'gWfLookup' to switch between wf)
inline float lsin(float deg){
  float a=deg;
  while(a>360) a-=360.0;
  while(a<0) a+=360.0;
  if (gWfLookup==1){ // sine gWfLookup
    int i=(int)((WAVEFORM_LOOKUP_TABLE_SIZE/360.0)*a);
    return SINE_LOOKUP_TABLE[i];
  }
  else if(gWfLookup==2){ // sine + 3rd gWfLookup
    int i=(int)((WAVEFORM_LOOKUP_TABLE_SIZE/360.0)*a);
    return SINE3RD_LOOKUP_TABLE[i];
  }
  else if(gWfLookup==3){ // saddle wf 
    int i=(int)((WAVEFORM_LOOKUP_TABLE_SIZE/360.0)*a);
    return SADDLE_LOOKUP_TABLE[i];
  }
  else if(gWfLookup==4){ // saddel wf
    int i=(int)((WAVEFORM_LOOKUP_TABLE_SIZE/360.0)*a);
    return SADDLE1_LOOKUP_TABLE[i];
  }
  else{ // sine math function (slow function to compute!)
    return (sin(deg*M_PI/180.0)+1.0)/2.0;
  }
}



void initLedC(){
  const int freq = 40000;

  ledcSetup(0, freq, gAdcResolution);
  ledcAttachPin(PWMw, 0);

  ledcSetup(1, freq, gAdcResolution);
  ledcAttachPin(PWMv, 1);
  
  ledcSetup(2, freq, gAdcResolution);
  ledcAttachPin(PWMu, 2);
  
}


byte m_color[5][3] = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}, {0, 0, 0} };
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

unsigned int gPulseCounter = 0;
unsigned int gPulseCounterP = 0;
int gPulseDirCounter = 0;

double gCompVelRpmTime; 

static uint32_t hall_sensor_value = 0;
static uint32_t hall_sensor_previous = 0;

inline void compVel(){
  if(gPulseCounterP!=gPulseCounter){
    double t=((uint32_t)XTHAL_GET_CCOUNT())*CLOCK_PERIOD_S;
    if((t-gCompVelRpmTime)<0) gCompVelRpmTime=gCompVelRpmTime-((double)(0xFFFFFFFF))*CLOCK_PERIOD_S;
    gCompVelRpm=(gPulseCounter-gPulseCounterP)/((t-gCompVelRpmTime))*60.0/(N_POLE_PAIRS*3);
    gPulseCounterP=gPulseCounter;
    gCompVelRpmTime=t;
  }
}


inline void hallReph(){
  // | H1 | 1 | 1 | 0 | 0 | 0 | 0 |
  // | H2 | 0 | 0 | 0 | 1 | 1 | 1 |
  // | H3 | 0 | 1 | 1 | 1 | 0 | 1 |
  // ------------------------------
  // | H  | 4 | 5 | 1 | 3 | 2 | 6 |
  // ------------------------------
  // | P  |352|032|096|160|224|288|

  if(gHallRephaseEn){
    float htp;
    /*
    if(hall_sensor_value==4)      htp=352.0;
    else if(hall_sensor_value==5) htp=032.0;
    else if(hall_sensor_value==1) htp=096.0;
    else if(hall_sensor_value==3) htp=160.0;
    else if(hall_sensor_value==2) htp=224.0;
    else if(hall_sensor_value==6) htp=288.0;
    */
    if(hall_sensor_value==4)      htp=330.0;
    else if(hall_sensor_value==5) htp=030.0;
    else if(hall_sensor_value==1) htp=090.0;
    else if(hall_sensor_value==3) htp=150.0;
    else if(hall_sensor_value==2) htp=210.0;
    else if(hall_sensor_value==6) htp=270.0;
    if(fabs(gPahseAng-htp)>120.0) gPahseAng=htp;
  }
  
}


void hallInterrupt1() {
  
  hall_sensor_value= (digitalRead(H1)*4 +digitalRead(H2)*2 +digitalRead(H3)*1);
  //Serial.printf("H1 %d %0.0f\n",hall_sensor_value,gPahseAng);


  hallReph();

  //update rev count on falling edge
  if(digitalRead(H1) == LOW){
    gPulseCounter++;
    
    if(digitalRead(H3)==1) gPulseDirCounter--;
    else gPulseDirCounter++;

    compVel();
  }
}


void hallInterrupt2() {

  hall_sensor_value= (digitalRead(H1)*4 +digitalRead(H2)*2 +digitalRead(H3)*1);
  //Serial.printf("H2 %d %0.0f\n",hall_sensor_value,gPahseAng);

  hallReph();

  
  //update rev count on falling edge
  if(digitalRead(H2) == LOW){
    gPulseCounter++;
    if(digitalRead(H1)==1) gPulseDirCounter--;
    else gPulseDirCounter++;
    
    compVel();
  }
  
}

void hallInterrupt3() {

  hall_sensor_value= (digitalRead(H1)*4 +digitalRead(H2)*2 +digitalRead(H3)*1);
  //Serial.printf("H3 %d %0.0f\n",hall_sensor_value,gPahseAng);

  hallReph();

  //update rev count on falling edge
  if(digitalRead(H3) == LOW){
    gPulseCounter++;
    if(digitalRead(H2)==1) gPulseDirCounter--;
    else gPulseDirCounter++;
    
    compVel();
  }
  
}




void setupHallSensorReader() {
  pinMode(H1, INPUT_PULLUP);
  pinMode(H2, INPUT_PULLUP);
  pinMode(H3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(H1), hallInterrupt1, CHANGE );
  attachInterrupt(digitalPinToInterrupt(H2), hallInterrupt2, CHANGE );
  attachInterrupt(digitalPinToInterrupt(H3), hallInterrupt3, CHANGE );
}



void setup() {

  // micro clock period computation
  CLOCK_PERIOD_S=1.0/XT_CLOCK_FREQ;

  
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 15, 4);

  //pin set GPIO pin mode
  
  //en
  pinMode(EnMot, OUTPUT);
  pinMode(EnBat, OUTPUT);
  
  //phases
  pinMode(PWMu, OUTPUT);
  pinMode(PWMv, OUTPUT);
  pinMode(PWMw, OUTPUT);

  //brk
  pinMode(BRK, OUTPUT);

  //button 2
  pinMode(Puls2, INPUT_PULLUP );

  //current sense
  pinMode(ADC_u, INPUT);
  pinMode(ADC_v, INPUT);
  pinMode(ADC_w, INPUT);
  pinMode(ADC_brake, INPUT);
  pinMode(ADC_batt, INPUT);

  // ADC settings
  analogReadResolution(10);
  analogSetAttenuation(ADC_0db);  

  // wf lookup tables
  generateWaveformLookupTables();

  //hall sensors interrupts
  setupHallSensorReader();

  //LED
  strip.begin();
  strip.setBrightness(5);
  strip.setLedColor(0,0xFFFFFF); strip.show(); delay(100);
  strip.setLedColor(0,0x000000); strip.show(); delay(100);
  strip.setLedColor(0,0xFFFFFF); strip.show(); delay(100);
  strip.setLedColor(0,0x000000); strip.show(); delay(100);
  strip.setLedColor(0,0xFFFFFF); strip.show(); delay(100);
  strip.setLedColor(0,0x000000); strip.show(); delay(100);
  
 
  // init ESP32 ledc PWM lib
  initLedC();
  
  
  /////////////////////////////////////////////////
  // launch DBS SERIAL - CORE 0
  /////////////////////////////////////////////////
  xTaskCreatePinnedToCore(
      TaskSerialDbg, /* Function to implement the task */
      "TaskHandleSerialDbg", /* Name of the task */
      4096,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &TaskHandleSerialDbg,  /* Task handle. */
      0); /* Core where the task should run */
 
  //enable battery
  digitalWrite(EnBat, HIGH);
  //enable motor
  digitalWrite(EnMot, HIGH); 
  
  
  Serial.println("INIT DONE");

  
}


////////////////////////////////////////////////////
// LOOP - CORE 1
////////////////////////////////////////////////////
double t=0.0;
void loop() {
  double r=((uint32_t)XTHAL_GET_CCOUNT())*CLOCK_PERIOD_S; 
  if((r-t)<0) t=t-((double)(0xFFFFFFFF))*CLOCK_PERIOD_S;
  gPahseAng+=(gDeg_s*(r-t));
  if(gPahseAng>360.0) gPahseAng-=360.0;
  if(gPahseAng<0.0) gPahseAng+=360.0;
  ledcWrite(0, (int)(gPowerSp * lsin(gPahseAng+000.0)) );
  ledcWrite(1, (int)(gPowerSp * lsin(gPahseAng+120.0)) );
  ledcWrite(2, (int)(gPowerSp * lsin(gPahseAng+240.0)) );
  
  gCompLoop_Hz=1.0/(r-t);
  t=r;
}



#define CS_N 2000
uint16_t CsI;
uint16_t CS_U[CS_N];
uint16_t CS_V[CS_N];
uint16_t CS_W[CS_N];
uint16_t CS_B[CS_N];
uint16_t V_BATT[CS_N];

uint32_t gPlotTime_ms=100;
int8_t gPlotType=-1;
double pt=0.0;
double t1=0.0;
void TaskSerialDbg(void *pvParameters){
  char cmd[64];
  int cmdI=0;
  while (1){
    double r=((uint32_t)XTHAL_GET_CCOUNT())*CLOCK_PERIOD_S; 
    if((r-t1)<0) t1=t1-((double)(0xFFFFFFFF))*CLOCK_PERIOD_S;
    if((r-pt)<0) pt=pt-((double)(0xFFFFFFFF))*CLOCK_PERIOD_S;

    while(Serial.available()>0){
      cmd[cmdI]=Serial.read();
      cmdI++;
    }
    cmd[cmdI]='\0';
    if(cmdI>0 && cmd[cmdI-1]=='\n'){
      cmd[cmdI-1]='\0';
      cmdI=0;
      
      if(strncmp(cmd,"R",1)==0) ESP.restart();

      //hall rephasing
      if(strncmp(cmd,"h",1)==0){
        if(atoi(&cmd[1])==1) gHallRephaseEn=true;
        else gHallRephaseEn=false;
      }
      //set power
      if(strncmp(cmd,"p",1)==0){
        gPowerSp=atof(&cmd[1])/100.0*pow(2,gAdcResolution);
      }
      //set velocity
      if(strncmp(cmd,"r",1)==0){
        gRpmSp=atof(&cmd[1]);
        gDeg_s=-gRpmSp/60.0*360.0*N_POLE_PAIRS;
      }
      //set lookup
      if(strncmp(cmd,"l",1)==0){
        gWfLookup=atoi(&cmd[1]);
      }
      //set plot time
      if(strncmp(cmd,"t",1)==0){
        gPlotTime_ms=atoi(&cmd[1]);
      }
      //set plot data
      if(strncmp(cmd,"d",1)==0){
        gPlotType=atoi(&cmd[1]);
        //reset serial plotter
        for(int i=0;i<500;i++){
          Serial1.print("0.000\t0.000\t0.000\n");
          vTaskDelay(5);
        }
        if(gPlotType==-1) Serial1.printf("--:\t--:\t--:\n");
        if(gPlotType== 0) Serial1.printf("--:\tloop_hz:\ttrd1_hz:\n");
        if(gPlotType== 1) Serial1.printf("--:\t-:\tI_U:\n");
        if(gPlotType== 2) Serial1.printf("--:\t-:\tI_V:\n");
        if(gPlotType== 3) Serial1.printf("--:\t-:\tI_W:\n");
        if(gPlotType== 4) Serial1.printf("--:\t-:\tI_B:\n");
        if(gPlotType== 5) Serial1.printf("--:\t--:\tV_BATT:\n");
        if(gPlotType== 6) Serial1.printf("I_U:\tI_V:\tI_V:\n");
        if(gPlotType== 7) Serial1.printf("--:\tref_vel:\tcurr_vel:\n");
        if(gPlotType== 8) Serial1.printf("--:\t--:\tangle:\n");
      }
      
      //if(strncmp(cmd,"q",1)==0){
      //  Serial.printf("REVS=%f gCompVelRpm=%frpm\n",(float)gPulseCounter/(N_POLE_PAIRS*3),gCompVelRpm);
      //}
      else{
        Serial.printf("[%s] POWER=%0.0fADC "
        "VEL=%0.2frpm "
        "HALL_REPH=%d "
        "LOOKUP=%s "
        "PLOT_TIME=%dms "
        "PLOT_TYPE=%s "
        "\n"
        ,cmd
        ,gPowerSp
        ,gRpmSp
        ,gHallRephaseEn
        ,gWfLookup==0?"SIN()":gWfLookup==1?"SINE":gWfLookup==2?"SINE+3RD":gWfLookup==3?"SADDLE":gWfLookup==4?"SADDLE1":"???"
        ,gPlotTime_ms
        ,gPlotType==-1?"NONE":gPlotType==0?"HZ":gPlotType==1?"I_U":gPlotType==2?"I_V":gPlotType==3?"I_W":gPlotType==4?"I_B":gPlotType==5?"V_BATT":gPlotType==6?"I_U;I_V;I_W":gPlotType==7?"VEL":gPlotType==8?"ANGLE":"??"
        );
      }
    }

    //Serial1 data plot
    //Serial.printf("%f %f\n",r,pt);
    if((r-pt)>=((double)gPlotTime_ms/1E3)){
      // loop freq plot
      if(gPlotType==0){
        Serial1.printf("%f\t%f\t%f\n",gCompLoop_Hz,gCompLoop_Hz,gCompThread1_Hz);
      }
      //phase currents plot
      else if(gPlotType==1 || gPlotType==2 || gPlotType==3 || gPlotType==4|| gPlotType==5|| gPlotType==6){
       float cu=0.0,cv=0.0,cw=0.0,cb,vb;
        for(int i=0;i<CS_N;i++){
          cu+=CS_U[i];
          cv+=CS_V[i];
          cw+=CS_W[i];
          cb+=CS_B[i];
          vb+=V_BATT[i];
        }
        cu/=CS_N;
        cv/=CS_N;
        cw/=CS_N;
        cb/=CS_N;
        vb/=CS_N;
        
        //plot one
        if(gPlotType==1) Serial1.printf("%f\t%f\t%f\n",cu,cu,cu);
        if(gPlotType==2) Serial1.printf("%f\t%f\t%f\n",cv,cv,cv);
        if(gPlotType==3) Serial1.printf("%f\t%f\t%f\n",cw,cw,cw);
        if(gPlotType==4) Serial1.printf("%f\t%f\t%f\n",cb,cb,cb);
        if(gPlotType==5) Serial1.printf("%f\t%f\t%f\n",vb,vb,vb);

        //plot all
        if(gPlotType==6) Serial1.printf("%f\t%f\t%f\n",cu,cv,cw);
     
      }

      // velocity setpoint - computed velocity plot
      if(gPlotType==7) Serial1.printf("%f\t%f\t%f\n",gRpmSp,gRpmSp,gCompVelRpm);
      
      // angle plot
      if(gPlotType==8){
        float rev=(float)gPulseDirCounter/(N_POLE_PAIRS*3);
        Serial1.printf("%f\t%f\t%f\n",rev,rev,rev);
      }
      
      pt=r;
    }

    

    //read current sense
    CS_U[CsI]=analogRead(ADC_u);
    CS_V[CsI]=analogRead(ADC_v);
    CS_W[CsI]=analogRead(ADC_w);
    CS_B[CsI]=analogRead(ADC_brake);
    V_BATT[CsI]=analogRead(ADC_batt);
    CsI=(CsI+1)%CS_N;
    
    feedTheDog(); //prevent wtd board reset

    gCompThread1_Hz=1.0/(r-t1);
    t1=r;

    
  }
 }
 
  
