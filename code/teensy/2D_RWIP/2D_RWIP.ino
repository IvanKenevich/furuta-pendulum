#include <Adafruit_MPU6050.h>
#include <Encoder.h>

//Loop Time Setup
#define IntervalMicro 2000 //interrupt timer interval in microseconds
const float Ts = IntervalMicro * 1e-6; //Sampling Time (sec)

//Constants to convert between radians and degrees
#define rad2deg 180 / M_PI
#define deg2rad M_PI / 180

//IMU Setup
sensors_event_t a, g, temp;
volatile float Omega, AccelX, AccelY; //Readings from IMU
const float as = 69 * deg2rad; //Approximate starting angle for gyro angle initialization
volatile float AngleA, AngleAFilt, AngleAFilt_m1,
         AngleG = as, AngleG_m1,
         AngleC = as, AngleC_m1; //Angles from each sensor as well as initialization for gyro integration
volatile float GyroBias; //Initial gyro bias, automatic
const float AngleABias = 0 * deg2rad; //Initial accel angle bias, manual because it's inconsistent due to noise
/*
   Time Constant for Complementary Filter.
   Increase to make the shape follow gyro more, decrease to make it move less from drift.
   Best to make as large as possible without causing signal to "drift" too much. 0.3-0.5 works well.
*/
//const float tauComp = 0.5;
const float alphaComp = 0.999; //tauComp / (tauComp + Ts);

//Encoder Setup
const float count2rad = 2 * M_PI / (48 * 10); //Constant to convert counts to radians
volatile float WheelSpeed, WheelSpeedFilt, WheelSpeedFilt_m1;
const int nCounts = ceil(0.025/Ts); //Number of encoder counts before averaging
volatile long Counts[nCounts] = {0};

//Motor Driver Setup
#define PWM1 30
#define PWM2 29
#define Vmax 12
volatile float Setpoint;

//Frequency Response
const float f = 10; //0.1-10 Hz for most DC motors
const float Amp = 6;

//State-space
const float a11 = 1.0002, a12 = 0.0019792, a13 = 1.70948e-06,
     a21 = 0.197121, a22 = 0.979344, a23 = 0.001699,
     a31 = -0.195641, a32 = 0.0205002, a33 = 0.98339;
const float b1 = -6.7461e-06, b2 = -0.00670385, b3 = 0.065547;

volatile float x1, x2, x3,
         x1_m1 = 0, x2_m1 = 0, x3_m1 = 0;
const float R = 0; //Setpoint (has to be 0)
const float AngleWhenTrying = 30 * deg2rad;
const float k1 = -111.046, k2 = -8.52735, k3 = -0.48368;

//Other
volatile unsigned long i = 0;
volatile unsigned long currenttime;
const float cutoffFreqLP = 50; //Cutoff frequency (Hz) low-pass
const float alphaLP = 1 / (2 * M_PI*cutoffFreqLP * Ts + 1);
volatile int ErrorDetect = 0;
volatile int Swingup = 0;
const float AngleWhenSwinging = 50  * deg2rad;

//Functions
float findBias();
void FlashForever();
void MotorControl(float);

//Constructors
IntervalTimer myTimer;
Encoder myEncoder(22, 23);
Adafruit_MPU6050 mpu;

/*================================ Timer Inturrupt. Put loop stuff here ================================*/
void ISR() {
  //Read Gyro and Accelerometer
  mpu.getEvent(&a, &g, &temp);
  Omega = g.gyro.z - GyroBias; //Remove gyro bias
  AccelX = a.acceleration.y;
  AccelY = a.acceleration.x;

  //Calculate angle from Accel
  AngleA = atan2(AccelY, AccelX) - M_PI / 2 + AngleABias;

  //Calculate angle from Gyro, only need for comparison to complementary version
  AngleG_m1 = AngleG;
  AngleG = AngleG_m1 + Omega * Ts;

  //Calculate angle from both with complementary filter
  AngleC_m1 = AngleC;
  AngleC = alphaComp * (AngleC_m1 + Omega * Ts) + (1 - alphaComp) * AngleA;

  //Obtain wheel speed using a "sampling time" of Ts*nCounts
  for (int i = nCounts - 1; i > 0; i--) {
    Counts[i] = Counts[i - 1];
  }
  Counts[0] = myEncoder.read();
  WheelSpeed = count2rad * (Counts[0] - Counts[nCounts - 1]) / ((nCounts - 1) * Ts);

  //  //Filter wheel speed
  //  WheelSpeedFilt_m1 = WheelSpeedFilt;
  //  WheelSpeedFilt = (1 - alphaLP) * WheelSpeed + alphaLP * WheelSpeedFilt_m1;
  //
  //  //Filter omega
  //  OmegaFilt_m1 = OmegaFilt;
  //  OmegaFilt = (1 - alphaLP) * Omega + alphaLP * OmegaFilt_m1;

  //Motor Control
  x1_m1 = AngleC, x2_m1 = Omega, x3_m1 = WheelSpeed;

  x1 = a11 * x1_m1 + a12 * x2_m1 + a13 * x3_m1 + b1 * (-k1 * x1_m1 - k2 * x2_m1 - k3 * x3_m1);
  x2 = a21 * x1_m1 + a22 * x2_m1 + a23 * x3_m1 + b2 * (-k1 * x1_m1 - k2 * x2_m1 - k3 * x3_m1);
  x3 = a31 * x1_m1 + a32 * x2_m1 + a33 * x3_m1 + b3 * (-k1 * x1_m1 - k2 * x2_m1 - k3 * x3_m1);

  if ( (fabs(AngleC) >= AngleWhenTrying) && (fabs(AngleC) <= AngleWhenSwinging)){ //Prep for swingup
    Setpoint = - copysign(Vmax, AngleC);
  } 
  else if (fabs(AngleC) <= AngleWhenTrying) { // balance
    Setpoint = R - k1 * x1 - k2 * x2 - k3 * x3;
  }
  else { // outside of effective range
    Setpoint = 0;
  }

  //  Setpoint = Amp*sin(2*M_PI*f*Ts*i); //For frequency response
  //  Setpoint = 1e-3*i; //Ramp
  //  Setpoint = 0;
  MotorControl(Setpoint);

//    Serial.println(micros()-currenttime); //Loop timer
//    currenttime = micros();

//  Serial.print(AngleC);
//  Serial.print("\t");
//  Serial.print(AccelX);
//  Serial.print("\t");
//  Serial.print(AccelY);
//  Serial.print("\t");
//  Serial.print(Omega);
//  Serial.print("\t");
//  Serial.print(Counts[0]);
//  Serial.print("\t");
//  Serial.print(Setpoint);
//
//  Serial.println();

  i++; //Loop counter
//  Serial.println(i);
}

/*================================ Setup ================================*/
void setup() {
  Serial.begin(250e3);
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  analogReadResolution(12);
  analogWriteResolution(12);

  //Try to initialize
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    FlashForever();
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG); //Set range for IMU, 250 might be too low sometimes
  Wire.setClock(1e6);

  GyroBias = findBias();

  digitalWrite(LED_BUILTIN, LOW);

  myTimer.priority(255); //Setting timer priority lower so it doesn't inturrupt encoder readings
  myTimer.begin(ISR, IntervalMicro);
}

void loop()
{
  delay(100); //print delay
//
  float AngleAPrint = AngleA,
        AngleGPrint = AngleG,
        AngleCPrint = AngleC,
        OmegaPrint = Omega,
        WheelSpeedPrint = WheelSpeed,
        SetpointPrint = Setpoint,
        AccelXPrint = AccelX,
        AccelYPrint = AccelY,
        WheelPosPrint = Counts[0]; //Print variables
//
//    Serial.print(AngleAPrint*rad2deg); //Angle from Accelerometer in Degrees
//    Serial.print("\t");
//  //
//  //  Serial.print(AngleGPrint * rad2deg); //Angle from Gyroscope in Degrees
//  //  Serial.print("\t");
//
//    Serial.print(AngleCPrint * rad2deg); //Angle from Complementary Filter in Degrees
//    Serial.print("\t");
//
//    Serial.print(WheelSpeedPrint*rad2deg);
//    Serial.print("\t");
////  
//  Serial.print(AccelXPrint);
//  Serial.print("\t");
//  Serial.print(AccelYPrint);
//  Serial.print("\t");
//  Serial.print(OmegaPrint);
//  Serial.print("\t");
//  Serial.print(WheelPosPrint);
//  Serial.print("\t");
//  Serial.print(SetpointPrint);
//
  Serial.println();
}

//Find Biases
float findBias() {
  int N = 1000;
  float Sum = 0;
  for (int i = 0; i < N; i++) {
    mpu.getEvent(&a, &g, &temp);
    Sum += g.gyro.z; //Gyro Z
  }
  return Sum / N;
}

//Error Flash Function
void FlashForever() {
  int LEDState = 0;
  while (true) {
    digitalWrite(LED_BUILTIN, LEDState);
    LEDState ^= 1;
    delay(200);
  }
}

//Motor Control
void MotorControl(float Setpoint) {
  if (fabs(Setpoint) >= Vmax) {
    Setpoint = copysign(Vmax, Setpoint);
  }
  int SetpointPWM = Setpoint / Vmax * 4095;
  if (SetpointPWM > 0) {
    analogWrite(PWM1, SetpointPWM);
    analogWrite(PWM2, LOW);
  }
  else {
    analogWrite(PWM1, LOW);
    analogWrite(PWM2, -SetpointPWM);
  }
}
