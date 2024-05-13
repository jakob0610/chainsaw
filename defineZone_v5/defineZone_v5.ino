#include <Wire.h>
//#include <MPU6050.h>
#include <math.h>

#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// LED pin for indicating work status
const int WORK_LED = 2; 

// LED pin for indicating brake status
const int BRAKE_LED = 3; 

// LED pin for indicating setting status
const int SETTING_LED = 4; 

// Button pin for resetting the system
const int RESET_BUTTON = 5; 

// Button pin for setting the system
const int SET_BUTTON = 6; 


// Motor pin for clockwise rotation
const int MOT_CLOCK = 7; 

// Motor pin for antilockwise rotation
const int MOT_ACLOCK = 8; 

// Button pin for running the chain
const int POWER_BUTTON = 9; 

// Alpha value for complementary filter
double ALPHA = 0.75; 

// Accelerometer angles in X and Y directions
double accAngleX, accAngleY; 

// Gyroscope angles in X, Y and Z directions
double gyroAngleX, gyroAngleY, gyroAngleZ; 

// Roll, pitch and yaw angles and their zero values
double roll, pitch, rollZero, pitchZero, yaw, yawZero;

// Combined value of roll, pitch and yaw
double combined_final;

// Reset value for yaw angle
double yawReset=0;

const double  offsetPitch=0; //not calculated data
const double  offsetRoll=0;    //not calculated data 
const double  offsetYaw=0;     //not calculated data

// Preset maximum and minimum values for pitch, roll, and yaw
const double PRESET_PITCH_MAX=80;
const double PRESET_PITCH_MIN=-80;
const double PRESET_ROLL_MAX=80;
const double PRESET_ROLL_MIN=-80;
const double PRESET_YAW_MAX=80;
const double PRESET_YAW_MIN=-80;

// Preset maximum combined value
const double PRESET_COMBINED_MAX=120;

// Final values for pitch, roll, and yaw after calculations
double pitch_final, roll_final, yaw_final;

// Maximum and minimum values for pitch, roll, and yaw after adjustments
 double Pitch_Max, Roll_Max, Yaw_Max;
 double Pitch_Min, Roll_Min, Yaw_Min;

// Maximum combined value after adjustments
double Combined_Max;

// Variables to hold time values for calculations
double previousTime;
double currentTime; 
double elapsedTime;


// Boolean variable to check if the chainsaw should be running
bool chainsawRunning;
bool emergency=false;

enum State 
{
  RUNNING,
  STANDBY,
  RESET,
  SET,
  EMERGENCY,
  STOP
};

State currentState=STANDBY;

void updateState(State newState) 
{
  currentState = newState;
}

void setup()
 {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
//
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-156);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(-14);
    mpu.setXAccelOffset(-3699);
    mpu.setYAccelOffset(-2519);
    mpu.setZAccelOffset(1391); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

    }
  // Set Pins for LED as Outputs
  pinMode(WORK_LED, OUTPUT);
  pinMode(BRAKE_LED, OUTPUT);
  pinMode(SETTING_LED, OUTPUT);

  // Set Pins for Button as Inputs
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(SET_BUTTON, INPUT_PULLUP);
    pinMode(POWER_BUTTON, INPUT_PULLUP);

  // Set Pins for Motor as Outputs
  pinMode(MOT_CLOCK, OUTPUT);
  pinMode(MOT_ACLOCK, OUTPUT);  

  //Set the Max & Min Values as Presets
      setMaxValues(PRESET_PITCH_MAX, PRESET_ROLL_MAX,PRESET_YAW_MAX,PRESET_COMBINED_MAX);
      setMinValues(PRESET_PITCH_MIN, PRESET_ROLL_MIN,PRESET_YAW_MIN);

  updateState(STANDBY);
  blink();
  chainsawRunning=false;
}


void loop()
{
  //Checking if a button is pressed
  checkButtonAndSetState(RESET_BUTTON, RESET);

  if(!emergency) //check if the emergency is activated
  {
    checkButtonAndSetState(SET_BUTTON, SET);
    checkButtonAndSetState(POWER_BUTTON, RUNNING);
  }

  //checking if the current Values of Pitch, Roll, Yaw & Combined, are in the set limits
  checkValueAndSetEmergency(pitch_final, Pitch_Min, Pitch_Max, "PITCH_OUT_OF_RANGE");
  checkValueAndSetEmergency(roll_final, Roll_Min, Roll_Max, "ROLL_OUT_OF_RANGE");
  checkValueAndSetEmergency(yaw_final, Yaw_Min, Yaw_Max, "YAW_OUT_OF_RANGE");
  checkValueAndSetEmergency(combined_final,0,Combined_Max,"COMBINED_OUT_OF_RANGE");



  switch(currentState)
  {
    case RUNNING:   //chainsaw is activ, trigger is pressed-> chain is moving
      chainsawRunning=true;
      digitalWrite(WORK_LED, HIGH);
      digitalWrite(BRAKE_LED, LOW);
      digitalWrite(SETTING_LED, LOW);
      setAngles();
      printAngles();
      startVibration();
      updateState(STANDBY);
    break;

    case STANDBY:  //chainsaw is activ, chain is not moving
      chainsawRunning=false;
      digitalWrite(WORK_LED, HIGH);
      digitalWrite(BRAKE_LED, LOW);
      digitalWrite(SETTING_LED, LOW);
      stopVibration();
    break;


    case RESET: //reset Button - reset YAW & goes to normal use
      chainsawRunning=false;
      emergency=false;
      digitalWrite(WORK_LED, LOW);
      digitalWrite(BRAKE_LED, HIGH);
      digitalWrite(SETTING_LED, HIGH);
      yawReset=0;
      setAngles();
      yawReset=yaw_final;
      setAngles();
      setMaxValues(PRESET_PITCH_MAX, PRESET_ROLL_MAX,PRESET_YAW_MAX,PRESET_COMBINED_MAX);
      setMinValues(PRESET_PITCH_MIN, PRESET_ROLL_MIN,PRESET_YAW_MIN);
      updateState(STANDBY);
      stopVibration();
    break;

    case SET: //Set-Button - defining ZONE
      chainsawRunning=false;
      stopVibration();
      digitalWrite(WORK_LED, LOW);
      digitalWrite(BRAKE_LED, LOW);
      digitalWrite(SETTING_LED, HIGH);


      if(Pitch_Max==PRESET_PITCH_MAX && Roll_Max==PRESET_ROLL_MAX && Yaw_Max==PRESET_YAW_MAX && Pitch_Min==PRESET_PITCH_MIN && Roll_Min==PRESET_ROLL_MIN && Yaw_Min==PRESET_YAW_MIN)
      {
        setMaxValues(5, 5, 5, 10);
        setMinValues(-5, -5, -5);
      }

      setAngles();
      checkValues();
      printMinMaxValues();
      //printAngles();
      updateState(STANDBY);
    break;

    case EMERGENCY: //Emergency-STOP
    emergency=true;
      chainsawRunning=false;
      stopVibration();
      digitalWrite(WORK_LED, LOW);
      digitalWrite(BRAKE_LED, HIGH);
      digitalWrite(SETTING_LED, LOW);
      //updateState(STOP);
    break;

  }
}

void setLEDs(int state) 
{
  digitalWrite(WORK_LED, state);
  digitalWrite(BRAKE_LED, state);
  digitalWrite(SETTING_LED, state);
}

void blink() 
{
  for(int i = 0; i < 2; i++) {
    setLEDs(HIGH);
    delay(200);
    setLEDs(LOW);
    delay(200);
  }
}

void setAngles()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
  { 

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetGyro(&gg, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw_final=(ypr[0] * RAD_TO_DEG) -yawReset;
        pitch_final=ypr[1] * RAD_TO_DEG*-1;
        roll_final=ypr[2] * RAD_TO_DEG;
        combined_final = abs(pitch_final) + abs(yaw_final) + 1.5 * abs(roll_final);

  }
}

void printWithTab(const String& text, float value) 
{
  Serial.print(text);
  Serial.print(value);
  Serial.println();
}

void printAngles()  //prints the current Values of Pitch, Roll & Yaw
{
  printWithTab("Pitch:", pitch_final);
  printWithTab("Roll:", roll_final);
  printWithTab("YAW:", yaw_final);
  printWithTab("COMBINED:", combined_final);
}

void printMinMaxValues() //prints the current Values of MIN & MAx Values for Pitch, Roll & Yaw
{
  printWithTab("Pitch_Max:", Pitch_Max);
  printWithTab("Roll_MAX:", Roll_Max);
  printWithTab("YAW_Max:", Yaw_Max);
  printWithTab("Combined_Max:", Combined_Max);
  printWithTab("Pitch_Min:", Pitch_Min);
  printWithTab("Roll_Min:", Roll_Min);
  printWithTab("YAW_Min:", Yaw_Min);
}

void setMaxValues(double pitch_max, double roll_max, double yaw_max, double combined_max)
{
  Combined_Max=combined_max;
  Pitch_Max=pitch_max;
  Roll_Max=roll_max;
  Yaw_Max=yaw_max;

}

void setMinValues(double pitch_min, double roll_min, double yaw_min)
{
  Pitch_Min=pitch_min;
  Roll_Min=roll_min;
  Yaw_Min=yaw_min;

}

void startVibration()
{
  digitalWrite(MOT_CLOCK,HIGH);
}

void stopVibration()
{
  digitalWrite(MOT_CLOCK,LOW);
  digitalWrite(MOT_ACLOCK,LOW);
}

void checkAndUpdatePitch() 
{
  if(pitch_final > Pitch_Max) Pitch_Max = pitch_final;
  if(pitch_final < Pitch_Min) Pitch_Min = pitch_final;
}

void checkAndUpdateRoll() 
{
  if(roll_final > Roll_Max) Roll_Max = roll_final;
  if(roll_final < Roll_Min) Roll_Min = roll_final;
}

void checkAndUpdateYaw() 
{
  if(yaw_final > Yaw_Max) Yaw_Max = yaw_final;
  if(yaw_final < Yaw_Min) Yaw_Min = yaw_final;
}

void checkValues() 
{
  checkAndUpdatePitch();
  checkAndUpdateRoll();
  checkAndUpdateYaw();
  Combined_Max = abs(Pitch_Max) + abs(Yaw_Max) + 1.5 * abs(Roll_Max);
}

void checkButtonAndSetState(int button, int state) 
{ //HIGH && last_state == true
  if(digitalRead(button) == LOW) 
  {
    updateState(state);
  }
}

void checkValueAndSetEmergency(float value, float min, float max, const char* message) 
{
  if(chainsawRunning && (value < min || value > max)) {
    Serial.println(message);
    Serial.print("Value:");
        Serial.println(value);
        Serial.print("MIN:");
        Serial.println(min);
        Serial.print("MAX:");
        Serial.println(max);
        Serial.println();
    updateState(EMERGENCY);
  }
}