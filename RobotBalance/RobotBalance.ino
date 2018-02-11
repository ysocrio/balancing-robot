

/*
Describe wiring layout here
*/
#include <Balance.h>
#include "I2Cdev.h"   //for MPU6050 library
#include "MPU6050_6Axis_MotionApps20.h"

//------------------------------------------------------------------------------
//         settings
//------------------------------------------------------------------------------
#define INITIAL_P 1
#define INITIAL_I 1
#define INITIAL_D 1
#define INITIAL_DESIRED_ANGLE 0
#define UPDATE_PERIOD 100
//#define SERIAL_ENABLE             //comment this out to disable serial
//for MPU6050
#define INTERRUPT_PIN 2  // use pin 2 on arbotixM
#define LED_PIN 13 //blonks when interrupt, from example
//obtained through IMU_0.ino
#define X_GYRO_OFFSET 220
#define Y_GYRO_OFFSET 76
#define Z_GYRO_OFFSET -85
#define X_ACCEL_OFFSET 1788
#define Y_ACCEL_OFFSET 1788
#define Z_ACCEL_OFFSET 1788
//------------------------------------------------------------------------------

//variables used for MPU6050
//interupts
volatile bool mpuInterrupt; // indicates whether MPU interrupt pin has gone high
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
uint8_t fifoBuffer[64]; // FIFO storage buffer
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
//until we read the sensor for the first time,  we assume it starts upright

//14 initial frame positions last two are ignored
int initialFrame[2][NUMBER_OF_SERVOS] =
{
  {512,512,205,818,512,512,512,512,512,512,512,808,512,512,00,00}, //angles
  {1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14 ,15,16}  //Servo ID Numbers
};

Balance Control(INITIAL_P,INITIAL_I,INITIAL_D,INITIAL_DESIRED_ANGLE);
ServoGroup Robot(initialFrame);

void dmpDataReady()
{
  mpuInterrupt = true;
};

void setup() {
  //initialize motors and sets frame
  Robot.ServosInitialize();

  //initialize sensor---------------------------------------------------------
  blinkState = false;
  dmpReady = false;
  mpuInterrupt = false;
  //initial setup
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  #ifdef SERIAL_ENABLE
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  #endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  #ifdef SERIAL_ENABLE
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  #endif
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);
  mpu.setXAccelOffset(X_ACCEL_OFFSET); // 1688 factory default for my test chip
  mpu.setYAccelOffset(Y_ACCEL_OFFSET); // 1688 factory default for my test chip
  mpu.setZAccelOffset(Z_ACCEL_OFFSET); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    #ifdef SERIAL_ENABLE
    Serial.println(F("Enabling DMP..."));
    #endif
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    #ifdef SERIAL_ENABLE
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    #endif
    //attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    #ifdef SERIAL_ENABLE
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
}

//------------------------------------------------------------------------------
// balancing starts here
//------------------------------------------------------------------------------

void loop() {
  //first set the robots frame
  Robot.SetAngles(initialFrame);

  //----------------------------------------------------------------------------
  //            Read Sensor data
  //----------------------------------------------------------------------------
  // if programming failed, don't try to do anything
  if (dmpReady){
    // wait for MPU interrupt or extra packet(s) available
    bool MPUeady = !(!mpuInterrupt && fifoCount < packetSize);
    if(MPUeady){
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
        pitch = ypr[1];
        //only compiles if SERIAL_LOGGING is defined above
        #ifdef SERIAL_ENABLE
        Serial.print("pitch andgle\t");
        Serial.println(pitch);
        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
      }
    }
  }

  //----------------------------------------------------------------------------
  //    calculate PID
  //----------------------------------------------------------------------------
  int torque = 0;
  torque = Control.UpdatePID(pitch);

  //----------------------------------------------------------------------------
  //    Set Torque
  //----------------------------------------------------------------------------

}
