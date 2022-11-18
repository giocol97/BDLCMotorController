//Constants

#include <Arduino.h>


#define ENABLE_PIN 21
#define LOG_RX 15
#define LOG_TX 4
#define LOG_BAUD 115200

//Hall sensors
#define HALL_U 5
#define HALL_V 17
#define HALL_W 18

//Motor constants
#define POLE_PAIRS 8
#define PHASE_RESISTANCE 1.42 //0.33
#define KV_RATING 383 //3660 //[rad/s/volt]  = 383 [rpm/volt]

//PWM 
#define PWM_FREQUENCY 20000

#define PWM_U 27
#define PWM_V 14
#define PWM_W 12

//Current sensing
#define SHUNT_RESISTOR 0.003
#define CURRENT_SENSING_GAIN 10

#define I_U 32
#define I_V 33
#define I_W 25

//per funzionalità
//costanti di controllo
#define ENCODER_PULSES_PER_REVOLUTION 16
#define MAX_SPEED 20 //rad/s
#define MIN_SPEED 16 //rad/s
#define SPEED_AFTER_RAMP 1 //rad/s

//costanti impulsi su rotaia
#define RAIL_START_PULSES 0
#define CONTROL_START_PULSES 25
#define BEGIN_RAMP_BEGIN_PULSES 50
#define END_RAMP_BEGIN_PULSES 400
#define END_RAMP_END_PULSES 500
#define RAIL_END_PULSES 600

float avgNoZero(float *arr, int size);
int radiansToImpulses(float rad);
float impulsesToRadians(int impulses);