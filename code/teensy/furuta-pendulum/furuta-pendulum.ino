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
volatile int motor_enabled = false;
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
#define PIN_SW_2    22
#define PIN_SW_3    37

#define PIN_MOTOR_ENBL PIN_SW_1
#define PIN_SWINGUP_ENBL PIN_SW_2

#define TIMER_CYCLE_MICROS 1000
const float Ts = 1e-6 * TIMER_CYCLE_MICROS;
IntervalTimer t1;

/*
 * Swing-up controller parameters
 */
enum state_enum {INIT, WAIT, MRIGHT, STOP, MLEFT, BALANCE, ERR} state = INIT;
volatile int swingup_triggered = false;
const float recovery_angle = 45;
const float swingup_disable_angle = (180.0 - recovery_angle) * deg2rad; // [rad] absolute value of arm2 angle where swingup controller switches off
const float balancing_angle_low = (180.0 - recovery_angle) * deg2rad;
const float balancing_angle_high = (180.0 + recovery_angle) * deg2rad;
const float swingup_trigger_low = 20.0 * deg2rad;
const float swingup_trigger_high = 340.0 * deg2rad;
const float swingup_move_halt_angle = 87 * deg2rad; // [rad] absolute value of arm2 angle at which swingup will stop arm1 to assist with swingup
const float swingup_reverse_angle = 95 * deg2rad;
const float swingup_move_voltage = - motor_supply_voltage * 0.7; // [V] Voltage swingup will use to run the motor
const float swingup_reverse_move_voltage = 5.0; // [V]
const float swingup_restart_speed = M_PI / 4; // [rad/s] absolute speed below which arm2 must be moving to restart swingup
unsigned long swingup_start_time;
const unsigned long swingup_attempt_duration = 750; // ms
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

Matrix<1,5> Kd = {-3.05653, 78.3661, -2.18186, 6.8087, 30.835};
Matrix<5,5> A = {1, 2.23068e-05, 0.000997985, -4.48714e-08, 0.000103481,
0, 1.00006, -2.01217e-06, 0.00099987, 0.000103351,
0, 0.0445818, 0.995969, -8.2238e-05, 0.206279,
0, 0.129307, -0.00402624, 0.999761, 0.206014,
0, -3.07747e-06, -0.000137323, 6.18916e-09, 0.984397};
Matrix<5,1, Array<5,1,volatile float> > B = {6.03142e-08, 6.02394e-08, 0.000180644, 0.000180417, 0.00173203};
Matrix<4,5> C = {1, 0, 0, 0, 0,
0, 1, 0, 0, 0,
0, 0, 1, 0, 0,
0, 0, 0, 1, 0};
Matrix<5,4> L = {0.0563782, 2.38433e-05, 0.0010064, 8.3569e-06,
-5.51398e-06, 0.0614293, 7.58378e-06, 0.00100945,
-0.0195241, 0.0463059, 0.0703032, 0.0228181,
-0.0194991, 0.131029, 0.0188741, 0.0740373,
-0.00394006, 0.00033814, 0.00388822, 0.0040204};

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

  state = WAIT;
  swingup_start_time = millis();
}

void isr_t1(void) {
  measure();

  control();

  // store previous values
  y_m1 = y;
  x_m1 = x;
//  x_m1(0) = y(0);
//  x_m1(1) = y(1);
//  x_m1(2) = y(2);
//  x_m1(3) = y(3);
  u_m1 = u;

  digitalWrite(PIN_TIMER_1, timer1_pin_state ^= 1);
}

void control() {
  switch (state) {
    case INIT:
      u(0) = 0;
      state = ERR; // we should've left this state back in setup()
      break;
    
    case WAIT:
      u(0) = 0;
      
      if ( swingup_triggered && ((enc2_pos < swingup_trigger_low) || (enc2_pos > swingup_trigger_high)) \
            && (fabs(enc2_speed) < swingup_restart_speed) ) {
        state = MRIGHT;
        swingup_start_time = millis();
      }
      break;

    case MRIGHT:
      u(0) = swingup_move_voltage;
      
      if (millis() - swingup_start_time > swingup_attempt_duration) {
        state = WAIT; // swingup timeout
      } else if ( (swingup_move_halt_angle <= enc2_pos) && (enc2_pos <= swingup_reverse_angle) ) {
        state = STOP; // stop moving
      }
      break;

    case STOP:
      u(0) = 0;
      
      if (enc2_pos < swingup_move_halt_angle) {
        state = WAIT; // we failed to swing up
      } else if ( (swingup_reverse_angle <= enc2_pos) && (enc2_pos <= balancing_angle_low) ) {
        state = MLEFT; // start moving the other direction
      }
      break;

    case MLEFT:
      u(0) = swingup_reverse_move_voltage;
      
      if (enc2_pos < swingup_reverse_angle) {
        state = WAIT; // we failed to swing up
      } else if ( (balancing_angle_low <= enc2_pos) && (enc2_pos <= balancing_angle_high) ) {
        state = BALANCE; // start balancing
        x(0) = y(0);
        x(1) = y(1);
        x(2) = y(2);
        x(3) = y(3);
        reference(0) = y(0);
      }
      break;

    case BALANCE:
      estimate();
      error = x;
      u = - Kd * error;
      if (fabs(u(0)) > motor_supply_voltage) {
        u(0) = copysign(motor_supply_voltage, u(0));
      } 
      else if (fabs(u(0)) < motor_minimum_voltage) {
        u(0) = 0;
      }

      /*if (stop_balancing) {
        state = WAIT; // stop balancing request
      } else*/ if ( (enc2_pos < balancing_angle_low) || (enc2_pos > balancing_angle_high) ) {
        state = WAIT; // we lost balance
      }
      break;

    case ERR:
      u(0) = 0;
      break;

    default:
      state = ERR; // you forgot to implement a state
      break;
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
  enc2_pos = 2 * M_PI * enc2_pos_raw / 8192.0;
  enc2_pos = f_fmod(enc2_pos, 2 * M_PI); // modulo with floored division

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
  y = {enc1_pos, enc2_pos - M_PI, enc1_speed, enc2_speed};
}

void estimate() {
  x = A * x_m1 + B * u_m1 + L * (y_m1 - C * x_m1);
//  x = A_1 * x_m1 + B * reference + L * y_m1;
//  x(4) = u(0) / Rm;
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

  digitalWrite(LED_PIN, square_source(1, 2) > 0 ? HIGH : LOW);

  if (digitalRead(PIN_MOTOR_ENBL) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }

  if (digitalRead(PIN_SWINGUP_ENBL) == HIGH) {
    swingup_triggered = true;
  } else {
    swingup_triggered = false;
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

/*
 * Floored division modulo. C uses truncated divisin modulo by default.
 * Background: https://en.wikipedia.org/wiki/Modulo
 * Source of implementation: https://www.alecjacobson.com/weblog/?p=1140
 */
float f_fmod(float a, float base) {
  return fmod(fmod(a, base) + base, base);
}
