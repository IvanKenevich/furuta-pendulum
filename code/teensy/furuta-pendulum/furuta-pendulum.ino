#include <Encoder.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

#define rad2deg (180.0 / M_PI)
#define deg2rad (M_PI / 180.0)

/*
  Encoder 1 (motor-mounted) variables
*/
#define PIN_ENC1_A   3 // encoder pins for the motor
#define PIN_ENC1_B   2
Encoder enc1(PIN_ENC1_A, PIN_ENC1_B);
volatile long enc1_pos_raw;               // counts read from the encoder
volatile float enc1_pos, enc1_speed;      // position and speed of the encoder in physical units
const size_t ENC1_POS_BUFF = 50;          // size of position buffer for velocity estimate
float enc1_pos_buff[ENC1_POS_BUFF] = {0}; // position buffer for velocity estimate

/*
  Encoder 2 (free arm-mounted) variables
*/
#define PIN_ENC2_A   1 // encoder pins for the free arm
#define PIN_ENC2_B   0
Encoder enc2(PIN_ENC2_A, PIN_ENC2_B);
volatile long enc2_pos_raw;               // counts read from the encoder
volatile float enc2_pos, enc2_speed;      // position and speed of the encoder in physical units
const size_t ENC2_POS_BUFF = 50;          // size of position buffer for velocity estimate
float enc2_pos_buff[ENC2_POS_BUFF] = {0}; // position buffer for velocity estimate

/*
  Motor variables
*/
#define PIN_PWM1    30 // PWM pins for motor driver
#define PIN_PWM2    29 
const float motor_supply_voltage = 12; // V
Matrix<1,1, Array<1,1,volatile float> > u = 0; // V
Matrix<1,1, Array<1,1,volatile float> > u_m1 = 0; // V
volatile float pwm_out = 0, pwm_out_copy = 0; // % PWM

/*
  Timing indicator pins
*/
#define PIN_TIMER_1   32 // timing indicator pin for isr_t1() function
#define PIN_TIMER_2   31 // timing indicator pin for loop() function
volatile char timer1_pin_state = 0, timer2_pin_state = 0;

/*
  Control switches
*/
#define PIN_SW_1    33 // motor enable pin
volatile int motor_enabled = false;
#define PIN_SW_2    39 // swingup start pin
#define PIN_SW_3    15

#define TIMER_CYCLE_MICROS 1000
const float Ts = 1e-6 * TIMER_CYCLE_MICROS;
IntervalTimer t1;

/*
 * Swing-up controller parameters
 */
const float swingup_disable_angle = (180.0 - 25) * deg2rad; // [rad] absolute value of arm2 angle where swingup controller switches off
const float swingup_move_halt_angle = 87 * deg2rad; // [rad] absolute value of arm2 angle at which swingup will stop arm1 to assist with swingup
const float swingup_move_voltage = 8; // [V] Voltage swingup will use to run the motor
int stop_swingup = false;

/*
 * Balancing controller parameters
 */
Matrix<5,1, Array<5,1,volatile float> > x = {}; //States
Matrix<5,1, Array<5,1,volatile float> > x_m1; //Previous states
Matrix<4,1, Array<4,1,volatile float> > y; // Measured states
Matrix<4,1, Array<4,1,volatile float> > y_m1; // Measured states

Matrix<1,5> Kd = {-0.0533779, 1.66421, -0.208186, 0.16789, -2.97875};
Matrix<5,5> A_1 = {-60.226, 1.90569e-05, -0.999009, 0, 0.000121262,
0, -62.8879, 0, -0.999, 0.000123879,
0.000135011, -41.1747, -21.9054, 14.9429, 0.240898,
0.000137924, -124.254, 33.7136, -39.1788, 0.24608,
0.00108994, -0.034024, 0.421568, -1.74347, 0.978896};
Matrix<5,1, Array<5,1,volatile float> > B = {0, 0, 0.00252934, 0.00258392, 0.0204193};
Matrix<5,4> L = {61.226, 0, 1,0,
0, 63.888, 0, 1,
0, 41.2113, 22.8871, -14.9435,
0, 124.374, -33.7323, 40.1777,
1.64263e-11, 0, -0.419319, 1.74004};

/*
 * Test square wave parameters`
 */
const unsigned long square_period = 1.0 * 1e6; // [usec] period of the square input wave
const float square_magnitude = 6.0; // [V] magnitude of the wave

elapsedMicros t_us; // teensyduino variable type that increments by itself
      
float setpoint;
            

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(PIN_TIMER_1, OUTPUT);
  pinMode(PIN_TIMER_2, OUTPUT);
  pinMode(PIN_SW_1, INPUT);
  pinMode(PIN_SW_2, INPUT);
  pinMode(PIN_SW_3, INPUT);

  Serial.begin(57600);

//  set_initial();
  
  t1.priority(255);
  t1.begin(isr_t1, TIMER_CYCLE_MICROS);

  digitalWrite(LED_BUILTIN, LOW);
}

void set_initial() {
  x = {0, 0, 0, 0, 0};
  x_m1 = {0.1, 0.1, 0.1, 0.1, 0.1};
  y = {0, 0, 0, 0};
  y_m1 = y;
  u = {0};
  u_m1 = {0};
}

void isr_t1(void) {
  measure();

  control();

  // store previous values
  y_m1 = y;
  x_m1 = x;
  u_m1 = u;

  digitalWrite(PIN_TIMER_1, timer1_pin_state ^= 1);
}

void control() {
  if (!stop_swingup) {
    if (fabs(enc2_pos) < swingup_move_halt_angle) {
      u(0) = 0; //swingup_move_voltage;
    } else if ( (swingup_move_halt_angle <= fabs(enc2_pos)) && (fabs(enc2_pos) <= swingup_disable_angle) ) {
      u(0) = 0;
    } else { // within the recovery window
      estimate();
      u(0) = 1; //u = - Kd * x;      
    }
  } else {
    u(0) = 0;
  }

//  if ( (t_us % square_period) < square_period / 2 ) {
//    u(0) = square_magnitude;
//  } else {
//    u(0) = 0;
//  }

  pwm_out = 100.0 * u(0) / motor_supply_voltage;
  if (motor_enabled) {
    write_pwm(pwm_out);
  } else {
    write_pwm(0);
  }
}

void write_pwm(float percent) {
  if (percent > 0) {
    analogWrite(PIN_PWM1, map(percent, 0, 100, 0, 256));
    analogWrite(PIN_PWM2, 0);
  }
  else if (percent < 0) {
    analogWrite(PIN_PWM1, 0);
    analogWrite(PIN_PWM2, map(percent, -100, 0, 256, 0));
  }
  else {
    analogWrite(PIN_PWM1, 0);
    analogWrite(PIN_PWM2, 0);
  }
}

void measure() {
  enc1_pos_raw = enc1.read(); // pulses
  enc2_pos_raw = enc2.read();
  enc1_pos = 2 * M_PI * enc1_pos_raw / 1632.67; // radians
  enc2_pos = 2 * M_PI * enc2_pos_raw / 8192.0;

  // shift over position buffer and add the new measurement
  for (int j = ENC1_POS_BUFF - 1; j > 0; j--) {
    enc1_pos_buff[j] = enc1_pos_buff[j - 1];
  }
  enc1_pos_buff[0] = enc1_pos;

  for (int j = ENC2_POS_BUFF - 1; j > 0; j--) {
    enc2_pos_buff[j] = enc2_pos_buff[j - 1];
  }
  enc2_pos_buff[0] = enc2_pos;

  // calculate speed as a secant line to position
  enc1_speed = (enc1_pos_buff[0] - enc1_pos_buff[ENC1_POS_BUFF - 1]) / ((ENC1_POS_BUFF - 1) * Ts);
  enc2_speed = (enc2_pos_buff[0] - enc2_pos_buff[ENC2_POS_BUFF - 1]) / ((ENC2_POS_BUFF - 1) * Ts);

  // put measurements into the vector
  y = {enc1_pos, enc2_pos, enc1_speed, enc2_speed};
}

void estimate() {
//  x = A_1 * x_m1 + B * u + L * y_m1;
  x = A_1 * x_m1 + B * u + L * y_m1;
}

void loop() {
  Serial.printf("y = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y(0), y(1), y(2), y(3));
  Serial.printf("y_m1 = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y_m1(0), y_m1(1), y_m1(2), y_m1(3));
  Serial.printf("x = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", x(0), x(1), x(2), x(3), x(4));
  Serial.printf("x_m1 = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", x_m1(0), x_m1(1), x_m1(2), x_m1(3), x_m1(4));
  Serial.printf("u = %4.3e\n", u(0));


  if (digitalRead(PIN_SW_1) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }
  
  delay(20);
  digitalWrite(PIN_TIMER_2, timer2_pin_state ^= 1);
}
