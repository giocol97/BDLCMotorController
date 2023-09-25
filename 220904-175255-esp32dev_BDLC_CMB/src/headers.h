// Constants

#include <Arduino.h>

#define CONFIG_NAMESPACE "config"

#define ENABLE_BAT 22

#define ENABLE_PIN 26
#define LOG_RX 15
#define LOG_TX 4
#define LOG_BAUD 115200

// Hall sensors
#define HALL_U 5
#define HALL_V 17
#define HALL_W 18

// Motor constants
#define POLE_PAIRS 8
#define PHASE_RESISTANCE 1.42 // 0.33
#define KV_RATING 383         // 3660 //[rad/s/volt]  = 383 [rpm/volt]

// PWM
#define PWM_FREQUENCY 25000

#define PWM_U 27
#define PWM_V 14
#define PWM_W 12

//quando freno è da abbassare TODO
#define IN_2 26

// Current sensing
#define SHUNT_RESISTOR 0.003
#define CURRENT_SENSING_GAIN 10

/*#define I_U 32
#define I_V 25
#define I_W 33*/

// per funzionalità
// costanti di controllo
#define ENCODER_PULSES_PER_REVOLUTION 16
#define MAX_SPEED 20       // rad/s
#define MIN_SPEED 16       // rad/s
#define SPEED_AFTER_RAMP 1 // rad/s

// costanti impulsi su rotaia
#define RAIL_START_PULSES 0
#define CONTROL_START_PULSES 25
#define BEGIN_RAMP_BEGIN_PULSES 50
#define END_RAMP_BEGIN_PULSES 400
#define END_RAMP_END_PULSES 500
#define RAIL_END_PULSES 600

// stati per macchina a stati

/*#define STATE_START 0
#define STATE_INACTIVE 1
#define STATE_SPINTA 2
#define STATE_FRENATA 3
#define STATE_QUASIFINECORSA 4
#define STATE_FINECORSA 5
#define STATE_INIZIO_RITORNO 6
#define STATE_RITORNO_VEL 7
#define STATE_RITORNO_TOR 8*/

#define STATE_START 0
#define STATE_INCORSA 1
#define STATE_STANDBY 2
#define STATE_INIZIO_RITORNO 3

#define DIRECTION_POSITIVE 1
#define DIRECTION_NEGATIVE -1
#define DIRECTION_UNDEFINED 0

#define STANDBY_TIMEOUT 30000

// valori default parametri
#define DEFAULT_VMAX 100
#define DEFAULT_VMAX_FRENATA 400
#define DEFAULT_VMIN_FRENATA 200
#define DEFAULT_C_FRENATA 3
#define DEFAULT_VMIN 50
#define DEFAULT_VTOCCO 25
#define DEFAULT_RAMP_DURATION UNDEFINED_VALUE
#define DEFAULT_PULSE_START RAIL_LENGTH_DEBUG * 0.1
#define DEFAULT_PULSE_STOP RAIL_LENGTH_DEBUG * 0.75
#define DEFAULT_PULSE_END RAIL_LENGTH_DEBUG * 0.9
#define DEFAULT_TEND 0.3
#define DEFAULT_TBRAKE 0.7
#define DEFAULT_TIMEOUT_DURATION 10000

float avgNoZero(float *arr, int size);
int radiansToImpulses(float rad);
float impulsesToRadians(int impulses);

// led control

void ledInit();
void ledDeInit();
void ledRed();
void ledGreen();
void ledBlue();
void ledMagenta();
void ledBlack();
