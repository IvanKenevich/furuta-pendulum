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
const size_t ENC1_POS_BUFF = 25;          // size of position buffer for velocity estimate
float enc1_pos_buff[ENC1_POS_BUFF] = {0}; // position buffer for velocity estimate

/*
  Encoder 2 (free arm-mounted) variables
*/
#define PIN_ENC2_A   1 // encoder pins for the free arm
#define PIN_ENC2_B   0
Encoder enc2(PIN_ENC2_A, PIN_ENC2_B);
volatile long enc2_pos_raw;               // counts read from the encoder
volatile float enc2_pos, enc2_speed;      // position and speed of the encoder in physical units
const size_t ENC2_POS_BUFF = 25;          // size of position buffer for velocity estimate
float enc2_pos_buff[ENC2_POS_BUFF] = {0}; // position buffer for velocity estimate

/*
  Motor variables
*/
#define PIN_PWM1    30 // PWM pins for motor driver
#define PIN_PWM2    29 
const float motor_supply_voltage = 12; // V
const float motor_minimum_voltage = 2; // V
Matrix<1,1, Array<1,1,volatile float> > u; // V
Matrix<1,1, Array<1,1,volatile float> > u_m1; // V
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
const float recovery_angle = 45.0;
const float swingup_disable_angle = (180.0 - recovery_angle) * deg2rad; // [rad] absolute value of arm2 angle where swingup controller switches off
const float swingup_move_halt_angle = 87 * deg2rad; // [rad] absolute value of arm2 angle at which swingup will stop arm1 to assist with swingup
const float swingup_move_voltage = -7; // [V] Voltage swingup will use to run the motor
unsigned long swingup_start_time;
const unsigned long swingup_attempt_duration = 500; // ms
int stop_swingup = false;

/*
 * Balancing controller parameters
 */
Matrix<5,1, Array<5,1,volatile float> > x = {}; //States
Matrix<5,1, Array<5,1,volatile float> > x_m1; //Previous states
Matrix<4,1, Array<4,1,volatile float> > y; // Measured states
Matrix<4,1, Array<4,1,volatile float> > y_m1; // Measured states
Matrix<5,1, Array<5,1,volatile float> > error = {}; //State Error
Matrix<5,1, Array<5,1,volatile float> > reference = {0, M_PI, 0, 0, 0}; // Reference

Matrix<1,5> Kd = {0, 146.309, -9.34953, 11.7675, 5.96349};
Matrix<5,5> A_1 = {0.984737, -0.000123578, 2.44495e-05, 6.84481e-06, 0.000113657,
-1.10744e-05, 0.983244, 2.54762e-05, 7.50217e-06, 0.000116109,
-0.0190516, -0.367918, 1.04173, 0.00287943, 0.21828,
-0.0194614, -0.375858, 0.0568023, 0.989061, 0.222974,
0.00542414, -2.98815, 0.181589, -0.249806, 0.796302};
Matrix<5,1, Array<5,1,volatile float> > B = {8.5044e-07, 8.68823e-07, 0.00252934, 0.00258392, 0.0204193};
Matrix<5,4> L = {0.015263, 1.96236e-05, 0.000974108, -1.6959e-05,
1.10744e-05, 0.0166913, -2.69495e-05, 0.000981951,
0.0190516, 0.0386643, -0.0368883, -0.0328491,
0.0194614, 0.121745, -0.0518583, -0.0200923,
-0.00542414, 0.000571218, 0.00732024, 0.00952212};

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

  set_initial();
  
  t1.priority(255);
  t1.begin(isr_t1, TIMER_CYCLE_MICROS);

  digitalWrite(LED_BUILTIN, LOW);
}

void set_initial() {
  x = {0, 0, 0, 0, 0};
  x_m1 = {0, 0, 0, 0, 0};
  y = {0, 0, 0, 0};
  y_m1 = y;
  u(0) = 0;
  u_m1(0) = 0;

  swingup_start_time = millis();
}

void isr_t1(void) {
  measure();

  control();

  // store previous values
  y_m1 = y;
  x_m1 = x;
  x_m1(0) = y(0);
  x_m1(1) = y(1);
  x_m1(2) = y(2);
  x_m1(3) = y(3);
  u_m1 = u;

  digitalWrite(PIN_TIMER_1, timer1_pin_state ^= 1);
}

void control() {
//  if (fabs(enc2_pos) < swingup_move_halt_angle) {
//    if (millis() - swingup_start_time < swingup_attempt_duration) {
//      u(0) = swingup_move_voltage;
//    } else {
//      u(0) = 0;
//    }
//  } else if ( (swingup_move_halt_angle <= fabs(enc2_pos)) && (fabs(enc2_pos) <= swingup_disable_angle) ) {
//    u(0) = 0;
//  } else 
    if ( ((180.0 - recovery_angle) * deg2rad <= enc2_pos) && (enc2_pos <= (180.0 + recovery_angle) * deg2rad) ) { // within the recovery window
      estimate();
      error = x - reference;
      u = - Kd * (x - reference);
      if (fabs(u(0)) > motor_supply_voltage) {
        u(0) = copysign(motor_supply_voltage, u(0));
      } else if (fabs(u(0)) < motor_minimum_voltage) {
        u(0) = 0;
      }
    } else {
      u(0) = 0; 
    }

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
//  enc2_pos = fmod(2 * M_PI * enc2_pos_raw / 8192.0, 2*M_PI);
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
  x = A_1 * x_m1 + L * y_m1;
}

void loop() {
//  Serial.printf("y = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y(0), y(1), y(2), y(3));
//  Serial.printf("y_m1 = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y_m1(0), y_m1(1), y_m1(2), y_m1(3));
//  Serial.printf("err = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", error(0), error(1), error(2), error(3), error(4));
//  Serial.printf("x_m1 = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", x_m1(0), x_m1(1), x_m1(2), x_m1(3), x_m1(4));
//  Serial.printf("u = %4.3e\n", u(0));

  Serial.printf("%4.3e, %4.3e, %4.3e, %4.3e, %4.3e"
                ",%4.3e, %4.3e, %4.3e, %4.3e"
                ",%4.3e, ",
                x(0), x(1), x(2), x(3), x(4),
                y(0), y(1), y(2), y(3),
                u(0));
  Serial.println(t_us);

  if (digitalRead(PIN_SW_1) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }
  
  delay(20);
  digitalWrite(PIN_TIMER_2, timer2_pin_state ^= 1);
}
