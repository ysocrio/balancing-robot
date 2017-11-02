
class ControlLoop
{
  private:
   //tuning variables
    double pVal;
    double iVal;
    double dVal;
   //desired value
    int setpoint;
   //output
    int outVal;
   //time keeping
    unsigned long timeInstance;
   //error
    int error;
    double errorSum; //for I term
  public:
    ControlLoop(double pInit, double iInit, double dInit, int desiredVal) {
     //initialize the object
     //tuningValues
      pVal = pInit;
      iVal = iInit;
      dVal = pInit;
     //time
      timeInstance = 0;
     //error
      error = 0;
    };
    void UpdatePID(int sensorVal) { //time is in millis, need to change so it is float/double in seconds
     //stuff that gets looped
      int previousTime = timeInstance;
      int previousError = error;
      error = setpoint - sensorVal;
      timeInstance = millis();
      int ellapsedTime = int(timeInstance - previousTime);   //(change of time)
      int errorChange = error - previousError;          //(change in error)
     //proportional term
      outVal += pVal * error;
     //integral term
      errorSum += ellapsedTime*error;
      outVal += errorSum*iVal;
     //derivative term
      if (ellapsedTime != 0 && previousError != 0) {
        int derivE = errorChange / ellapsedTime;
        outVal += derivE * dVal;
      }
    };
};

void setup() {
  ControlLoop balance(1,1,1,0);
}

void loop() {
}

