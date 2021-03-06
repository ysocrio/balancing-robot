/*
  Describe wiring layout here
  -dynamixel configuration:
  -MPU6050
   Vin to 5v
   gnd to gnd
*/

//fifo buffer speed is now set to 0x04 was 0x01 (in mpu6050_6Axis_MotionApps20.h)

//------------------------------------------------------------------------------
//         settings
//------------------------------------------------------------------------------
#define INITIAL_K 1
#define INITIAL_P 0
#define INITIAL_I 0
#define INITIAL_D 0
#define INITIAL_DESIRED_ANGLE 3
#define UPDATE_PERIOD 100
#define SERIAL_ENABLE 1            //comment this out to disable serial

//for MPU6050
#define INTERRUPT_PIN 2  // use pin 2 on arbotixM
#define LED_PIN 0 //blinks when interrupt, from example

//obtained through IMU_0.ino
#define X_GYRO_OFFSET 220
#define Y_GYRO_OFFSET 76
#define Z_GYRO_OFFSET -85
#define X_ACCEL_OFFSET 1788
#define Y_ACCEL_OFFSET 1788
#define Z_ACCEL_OFFSET 1788

//how long to wait befor begining
#define MPU_SETTLING_TIME_DELAY 6000 //6 seconds

//------------------------------------------------------------------------------

#include <Balance.h>
//https://github.com/jrowberg/i2cdevlib/tree/master/Arduino
#include "I2Cdev.h"   //for MPU6050 library
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define OUTPUT_READABLE_YAWPITCHROLL

//variables used for MPU6050
//interupts
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady();      //function that runs when MPU6050 interrupts code
//variables for MPU
MPU6050 mpu;            //class that represents sensor
bool blinkState;
int updatePeriod;
// MPU control/status vars
bool dmpReady;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO

//arduino restarting due to not enough ram? this uses a quarter of ram, lets shrink this
//potential solution: chang DMP output rate, then fifoBuffer size in arduino
uint8_t fifoBuffer[64]; // FIFO storage buffer64
//containers for MPU6050
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
// orientation/motion vars
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pitch = INITIAL_DESIRED_ANGLE;
//decides when to start
bool vertical = false;
unsigned long startTime;

//14 initial frame positions last two are ignored
int initialFrame[2][NUMBER_OF_SERVOS] =
{
  {512, 512, 205, 818, 512, 512, 512, 512, 512, 512, 512, 808, 512, 512, 00, 00}, //angles
  {3,  2,  1,  4,  5,  6,  7,  8,  9,  17, 11, 12, 13, 18 , 15, 16} //Servo ID Numbers
  //one is not responding
};

double torque = 0;
int convertedTorque = 0;


//object definitions
Balance Control(INITIAL_K, INITIAL_P, INITIAL_I, INITIAL_D, INITIAL_DESIRED_ANGLE);
ServoGroup Robot(initialFrame);
float k = 1;
float p = 0;
float i = 0;
float d = 0;
float initialAngle = 0;

//called when MPU6050 triggers INTERRUPT_PIN
void dmpDataReady()
{
  mpuInterrupt = true;
};

void setup() {

#ifdef SERIAL_ENABLE
  Serial.begin(115200);
  //Ask user for PID values
  Serial.setTimeout(100000);
  Serial.println("input K value:");
  k = Serial.readStringUntil('\n').toDouble();
  Serial.println("input P value:");
  p = Serial.readStringUntil('\n').toDouble();
  Serial.println("input I value:");
  i = Serial.readStringUntil('\n').toDouble();
  Serial.println("input D value:");
  d = Serial.readStringUntil('\n').toDouble();
  Serial.println("input Desired Angle value:");
  initialAngle = Serial.readStringUntil('\n').toDouble();
  //Serial.println(F("Initializing I2C devices..."));
  Control.SetPID(k, p, i, d);
  Control.SetDesiredVal(initialAngle);
#endif
  //initialize robot----------------------------------------------------------
  Robot.ServosInitialize();
  Robot.SetSpeeds(0, 0);
  //first set the robots frame
  Robot.SetAnglesAll(initialFrame);

  //initialize sensor---------------------------------------------------------
  blinkState = false;
  dmpReady = false;
  //initial setup



  // join I2C bus (I2Cdev library doesn't do this automatically)T
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
#ifdef SERIAL_ENABLE
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //mpu.testConnection();
  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here
  mpu.setXGyroOffset(130);
  mpu.setYGyroOffset(10);
  mpu.setZGyroOffset(15);
  mpu.setXAccelOffset(-2495); // 1688 factory default for my test chip
  mpu.setYAccelOffset(-1524); // 1688 factory default for my test chip
  mpu.setZAccelOffset(1410); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
#ifdef SERIAL_ENABLE
    //Serial.println(F("Enabling DMP..."));
#endif
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
#ifdef SERIAL_ENABLE
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
#endif
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef SERIAL_ENABLE
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
#ifdef SERIAL_ENABLE
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //only starts balancing a fixed time after this point
  startTime = millis();
}
void loop() {
  //check to see if controller is sending a desired arm angle or turnVal
  float armAngle = 0;
  float turnVal = 0;
  if (Serial.available() > 0)
  {
    //sent data will be in the form "K P I D\n"
    //all values are floats
    //following lines will convert above string to floats
    float Knew = Serial.parseFloat();
    float Pnew = Serial.parseFloat();
    float Inew = Serial.parseFloat();
    float Dnew = Serial.parseFloat();
    Control.SetPIDLive(Knew,Pnew,Inew,Dnew);
    /*
    armAngle = Serial.parseFloat();
    turnVal = Serial.parseFloat();
    */
  }

  //----------------------------------------------------------------------------
  //            Read Sensor data
  //----------------------------------------------------------------------------
  // if programming failed, don't try to do anything
  if (dmpReady) {
    // wait for MPU interrupt or extra packet(s) available
    bool MPUready = !(!mpuInterrupt && fifoCount < packetSize);
    if (MPUready) {
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
#ifdef SERIAL_ENABLE
        Serial.println(F("FIFO overflow!"));
#endif
      }
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();            //change this so it has a watchdog

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        pitch = ypr[1] * 180 / M_PI;



        //only compiles if SERIAL_ENABLE is defined above
#ifdef SERIAL_ENABLE
        Serial.print(pitch);
        Serial.print(" ");
        Serial.println(initialAngle);

#endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        //------------------------------------------------------------------------------
        // balancing starts here
        //--------------------------------------------------------------
        //

        //only begins when robot is within one degree of vertical
        if (millis() - startTime > MPU_SETTLING_TIME_DELAY)
        {
          if ((pitch - initialAngle > -.1) && (pitch - initialAngle < .1))
          {
            vertical = true;
          }
          if (vertical) {
            //updateFrame with armAngle (to be added)

            //Set frame
            Robot.SetAngles(initialFrame);
            //calculate PID then get torque from PID loop and constrain from zero to max
            noInterrupts();
            torque = constrain(Control.UpdatePID(pitch), -MAX_TORQUE, MAX_TORQUE);
            interrupts();

            //Set Output
            //maps torque because torque is set from 0 to 1023
            convertedTorque = map(torque, 0, MAX_TORQUE, 0, 1023);
            //apply turning info then set speed (to be added)
            noInterrupts();
            Robot.SetSpeeds(-convertedTorque, convertedTorque); //first input works, second does not
            interrupts();
          }
        }
      }
    }
  }
}
