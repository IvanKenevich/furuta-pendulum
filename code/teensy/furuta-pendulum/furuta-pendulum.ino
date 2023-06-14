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
const float recovery_angle = 30.0;
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

Matrix<1,5> Kd = {-0.0533779, 1.66421, -0.208186, 0.16789, -2.97875};
Matrix<5,5> A_1 = {0.99081, -4.25819e-07, 1.82515e-05, 1.83258e-05, 0.000121262,
-1.03315e-05, 0.989988, 1.89121e-05, 1.89911e-05, 0.000123879,
-0.018377, -0.0019144, 1.02737, 0.0355309, 0.240898,
-0.0187722, -0.00195588, 0.0364882, 1.02794, 0.24608,
0.00684957, -0.0346973, -0.00684622, -0.0147657, 0.978896};
Matrix<5,1, Array<5,1,volatile float> > B = {8.5044e-07, 8.68823e-07, 0.00252934, 0.00258392, 0.0204193};
Matrix<5,4> L = {0.0091896, 1.94827e-05, 0.000972532, -1.85752e-05,
1.03778e-05, 0.0100725, -2.83276e-05, 0.00098054,
0.018512, 0.038516, -0.0456531, -0.0361613,
0.0189101, 0.121594, -0.0551647, -0.0290037,
-0.00575962, 0.000673367, 0.00909588, 0.0113378};

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
  if (fabs(enc2_pos) < swingup_move_halt_angle) {
    if (millis() - swingup_start_time < swingup_attempt_duration) {
      u(0) = swingup_move_voltage;
    } else {
      u(0) = 0;
    }
  } else if ( (swingup_move_halt_angle <= fabs(enc2_pos)) && (fabs(enc2_pos) <= swingup_disable_angle) ) {
    u(0) = 0;
  } else if ( ((180.0 - recovery_angle) * deg2rad <= fabs(enc2_pos)) && (fabs(enc2_pos) <= (180.0 + recovery_angle) * deg2rad) ) { // within the recovery window
    estimate();
    error = x - reference;
    u = - Kd * error;
    if (fabs(u(0)) > motor_supply_voltage) {
      u(0) = copysign(motor_supply_voltage, u(0));
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
  reference(0) = y(0); // set arm1 reference to its current value
}

void estimate() {
  x = A_1 * x_m1 + B * u + L * y_m1;
}

void loop() {
//  Serial.printf("y = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y(0), y(1), y(2), y(3));
//  Serial.printf("y_m1 = {%4.3e, %4.3e, %4.3e, %4.3e}\n", y_m1(0), y_m1(1), y_m1(2), y_m1(3));
//  Serial.printf("x = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", x(0), x(1), x(2), x(3), x(4));
//  Serial.printf("x_m1 = {%4.3e, %4.3e, %4.3e, %4.3e, %4.3e}\n", x_m1(0), x_m1(1), x_m1(2), x_m1(3), x_m1(4));
//  Serial.printf("u = %4.3e\n", u(0));

//  Serial.printf("%f\t%f\t%f\t%f\n", -87.0, 87.0, rad2deg * enc2_pos, rad2deg * x(1));

  Serial.printf("%f\t%f\n", u(0), x(4));

  if (digitalRead(PIN_SW_1) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }
  
  delay(20);
  digitalWrite(PIN_TIMER_2, timer2_pin_state ^= 1);
}
