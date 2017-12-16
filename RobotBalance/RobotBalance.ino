#include <Balance.h>
#define INITIAL_P 1
#define INITIAL_I 1
#define INITIAL_D 1
#define INITIAL_DESIRED_ANGLE 0
#define UPDATE_PERIOD 100
Balance Control(INITIAL_P,INITIAL_I,INITIAL_D,INITIAL_DESIRED_ANGLE);
Sensor SensorBalance(UPDATE_PERIOD);
int initialFrame[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//16 initial frame positions

void setup() {
  //initialize motors
  Control.ServosInitialize();
  //SetFrame
  Control.SetFrame(initialFrame);
  //initialize sensor
  SensorBalance.Connect();
}

void loop() {
  //setFrame (optional for now)
  //Control.SetFrame(intialFrame);
  //check sensor
  SensorBalance.Update();
  //calculate wheel SetSpeeds //set wheelspeed
  int wheelSpeed = Control.UpdatePID(SensorBalance.GetPitchAngle());
  //set wheelspeed
  Control.SetSpeeds(wheelSpeed, -wheelSpeed);
}
