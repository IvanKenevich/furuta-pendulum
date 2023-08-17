#include <Encoder.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
#define LED_PIN 23

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
#define PIN_PWM1    35 // PWM pins for motor driver
#define PIN_PWM2    36 
const float motor_supply_voltage = 12; // V
const float motor_minimum_voltage = 0; // V
const float Rm = 9.0; // Ohm - motor resistance
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
#define PIN_SW_1    18
#define PIN_SW_2    13
#define PIN_SW_3    37

#define PIN_MOTOR_ENBL PIN_SW_1
volatile int motor_enabled = false;

#define TIMER_CYCLE_MICROS 1000
const float Ts = 1e-6 * TIMER_CYCLE_MICROS;
IntervalTimer t1;

/*
 * Swing-up controller parameters
 */
const float recovery_angle = 45;
const float swingup_disable_angle = (180.0 - recovery_angle) * deg2rad; // [rad] absolute value of arm2 angle where swingup controller switches off
const float swingup_move_halt_angle = 87 * deg2rad; // [rad] absolute value of arm2 angle at which swingup will stop arm1 to assist with swingup
const float swingup_move_voltage = - motor_supply_voltage * 0.7; // [V] Voltage swingup will use to run the motor
unsigned long swingup_start_time;
const unsigned long swingup_attempt_duration = 750; // ms
int stop_swingup = false;
int swingup_state = 0;

/*
 * Balancing controller parameters
 */
Matrix<5,1, Array<5,1,volatile float> > x = {}; //States
Matrix<5,1, Array<5,1,volatile float> > x_m1; //Previous states
Matrix<4,1, Array<4,1,volatile float> > y; // Measured states
Matrix<4,1, Array<4,1,volatile float> > y_m1; // Measured states
Matrix<5,1, Array<5,1,volatile float> > error = {}; //State Error
Matrix<5,1, Array<5,1,volatile float> > reference = {0, M_PI, 0, 0, 0}; // Reference

Matrix<1,5> Kd = {-83.5919, 534.127, -23.4747, 42.1621, 71.069};
Matrix<5,5> A_1 = {0.990566, -3.00376e-05, 2.77617e-06, -1.02104e-06, 9.39757e-05,
-1.76327e-05, 0.989689, 3.15064e-06, -6.63591e-07, 9.60036e-05,
-0.029462, -0.0910144, 0.998073, -0.00456954, 0.183266,
-0.0300947, -0.0929786, 0.00679055, 0.986758, 0.187207,
0.146129, -0.925135, 0.0405629, -0.0731225, 0.861305};
Matrix<5,1, Array<5,1,volatile float> > B = {5.71406e-08, 5.83758e-08, 0.00017114, 0.000174833, 0.00173203};
Matrix<5,4> L = {0.00943889, 2.00922e-05, 0.000996656, -1.49532e-06,
2.25125e-05, 0.0103418, -3.73018e-06, 0.000997879,
0.0437679, 0.0407232, 0.00212475, -0.00285345,
0.0447094, 0.123848, -0.0065879, 0.0052443,
-0.00134594, 9.65637e-06, -4.14945e-05, 9.6651e-05};

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
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(PIN_SW_1, INPUT_PULLUP);
  pinMode(PIN_SW_2, INPUT_PULLUP);
  pinMode(PIN_SW_3, INPUT_PULLUP);

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
  if (fabs(enc2_pos) < swingup_move_halt_angle) {
    if (millis() - swingup_start_time < swingup_attempt_duration) {
      swingup_state = 1;
      u(0) = swingup_move_voltage;
    } 
    else {
      u(0) = 0;
      swingup_state = 10;
    }
  } 
  else if ( (fabs(enc2_pos) >= swingup_move_halt_angle) && (fabs(enc2_pos) < 95 * deg2rad)) {
    swingup_state = 2;
    u(0) = 0; 
  } 
  else if ( fabs((enc2_pos) >= 95 * deg2rad) && (fabs(enc2_pos) <= swingup_disable_angle)){
    swingup_state = 3;
    x(0) = y(0);
    x(1) = y(1);
    x(2) = y(2);
    x(3) = y(3);
    
    u(0) = 5;

    reference(0) = y(0);
  }
//  else if ( (90 * deg2rad <= enc2_pos && enc2_pos < (180.0 - recovery_angle) * deg2rad) || ((180.0 + recovery_angle) * deg2rad < enc2_pos && enc2_pos <= 270 * deg2rad ) ) { // between 90 and recovery (on both sides)
//    // attempt to feed better ICs to recovery window
//    x(0) = y(0);
//    x(1) = y(1);
//    x(2) = y(2);
//    x(3) = y(3);
//    
//    u(0) = 0;
//  }
  else if ( ((180.0 - recovery_angle) * deg2rad <= enc2_pos) && (enc2_pos <= (180.0 + recovery_angle) * deg2rad) ) { // within the recovery window
    swingup_state = 0;
    estimate();
    error = x - reference;
    u = - Kd * (x - reference);
    if (fabs(u(0)) > motor_supply_voltage) {
      u(0) = copysign(motor_supply_voltage, u(0));
    } 
    else if (fabs(u(0)) < motor_minimum_voltage) {
      u(0) = 0;
    }
  } 
  else {
    swingup_state = -1;
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
//  enc1_pos = 2 * M_PI * enc1_pos_raw / 1632.67; // radians - Ivan's motor
  enc1_pos = 2 * M_PI * enc1_pos_raw / (48 * 10) * 1.0784; // radians - Jacob's motor
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
  x(4) = u(0) / Rm;
}

void loop() {
//  Serial.printf("y = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y(0), y(1), y(2), y(3));
//  Serial.printf("y_m1 = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y_m1(0), y_m1(1), y_m1(2), y_m1(3));
//  Serial.printf("err = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", error(0), error(1), error(2), error(3), error(4));
//  Serial.printf("x_m1 = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", x_m1(0), x_m1(1), x_m1(2), x_m1(3), x_m1(4));
//  Serial.printf("u = %4.3e\n", u(0));
//
//  Serial.printf("% 4.3e, % 4.3e, % 4.3e, % 4.3e, % 4.3e"
//                ",% 4.3e, % 4.3e, % 4.3e, % 4.3e"
//                ",% 4.3e, ",
//                x(0), x(1), x(2), x(3), x(4),
//                y(0), y(1), y(2), y(3),
//                u(0));
//  Serial.println(t_us);

  Serial.printf("% 4.3e, %d\n", y(1), swingup_state);

  digitalWrite(LED_PIN, square_source(1, 2) > 0 ? HIGH : LOW);

  if (digitalRead(PIN_SW_1) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }
  
  delay(10);
  digitalWrite(PIN_TIMER_2, timer2_pin_state ^= 1);
}

float sin_source(float amp, float freq_hz) {
  float freq_rad = freq_hz * 2 * M_PI;
  return amp * sin(freq_rad * (t_us * 1e-6));
}

float square_source(float amp, float freq_hz) {
  float period = 1 / freq_hz;
  float modval = fmod((t_us * 1e-6), period);
  return (modval > (period / 2)) ? amp : -amp;
}
