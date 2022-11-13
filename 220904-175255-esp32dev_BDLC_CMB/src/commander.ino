/*#include <SimpleFOC.h>

// include commander interface
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup(){
  ....
  // add the motor to the commander interface
  // The letter (here 'M') you will provide to the SimpleFOCStudio
  command.add('M',doMotor,'motor');
  // tell the motor to use the monitoring
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable monitor at first - optional
  ...

}
void loop(){
  ....

  ....
  // real-time monitoring calls
  motor.monitor();
  // real-time commander calls
  command.run();
}*/