#include <Encoder.h>

#define rad2deg (180.0 / M_PI)
#define deg2rad (M_PI / 180.0)

/*
  Encoder 1 (motor-mounted) variables
*/
#define PIN_ENC1_A   2 // encoder pins for the motor
#define PIN_ENC1_B   3
Encoder enc1(PIN_ENC1_A, PIN_ENC1_B);
volatile long enc1_pos_raw;               // counts read from the encoder
volatile float enc1_pos, enc1_speed;      // position and speed of the encoder in physical units
const size_t ENC1_POS_BUFF = 50;          // size of position buffer for velocity estimate
float enc1_pos_buff[ENC1_POS_BUFF] = {0}; // position buffer for velocity estimate

/*
  Encoder 2 (free arm-mounted) variables
*/
#define PIN_ENC2_A   0 // encoder pins for the free arm
#define PIN_ENC2_B   1
Encoder enc2(PIN_ENC2_A, PIN_ENC2_B);
volatile long enc2_pos_raw;               // counts read from the encoder
volatile float enc2_pos, enc2_speed;      // position and speed of the encoder in physical units
const size_t ENC2_POS_BUFF = 50;          // size of position buffer for velocity estimate
float enc2_pos_buff[ENC2_POS_BUFF] = {0}; // position buffer for velocity estimate

/*
  Motor variables
*/
#define PIN_PWM1    29 // PWM pins for motor driver
#define PIN_PWM2    30 
const float motor_supply_voltage = 12; // V
volatile float motor_control = 0; // V
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

#define TIMER_CYCLE_MICROS 100
const float Ts = 1e-6 * TIMER_CYCLE_MICROS;
IntervalTimer t1;

/*
 * Swing-up controller parameters
 */
const float swingup_disable_angle = (180.0 - 20) * deg2rad; // [rad] absolute value of arm2 angle where swingup controller switches off
volatile enum swingup_state_t {START, MOVE, HALT, END} swingup_state = END;
volatile unsigned long swingup_t_start, swingup_t_halt, swingup_t_end;
const float swingup_move_voltage = 6; // [V] Voltage swingup will use to run the motor
const unsigned long swingup_move_duration = 1 * 1e6, // [us] how long the swingup will spin arm at constant speed before stopping
                    swingup_halt_duration = 1 * 1e6; // [us] how long the swingup will wait for the arm to swing up before giving up

elapsedMicros t_us; // teensyduino variable type that increments by itself
const float input_freq = 8, // Hz
            input_amp  = 0; // V
            
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
  
  t1.begin(isr_t1, TIMER_CYCLE_MICROS);

  digitalWrite(LED_BUILTIN, LOW);
}

void isr_t1(void) {
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


  if (fabs(enc2_pos) < swingup_disable_angle) {
    motor_control = run_swingup();
  } else {
    // balancing controller
    motor_control = 0;
  }

  pwm_out = 100.0 * motor_control / motor_supply_voltage;
  if (motor_enabled) {
    write_pwm(pwm_out);
  } else {
    write_pwm(0);
  }

  digitalWrite(PIN_TIMER_1, timer1_pin_state ^= 1);
}

float run_swingup() {
  float V;
  switch (swingup_state) {
    case START:
      swingup_t_start = t_us;
      swingup_t_halt = swingup_t_start + swingup_move_duration;
      swingup_t_end = swingup_t_halt + swingup_halt_duration;
      V = 0;
      
      swingup_state = MOVE;
      break;
      
    case MOVE:
      V = swingup_move_voltage;

      if (t_us > swingup_t_halt) {swingup_state = HALT;}
      break;

    case HALT:
      V = -swingup_move_voltage;

      if (t_us > swingup_t_end || fabs(enc2_pos) >= swingup_disable_angle) 
        {swingup_state = END;}
      break;

    case END:
      V = 0;
      
      break;

    default:
      V = 0;
      break;
  }
  return V;
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

void loop() {
  Serial.print(rad2deg * enc2_pos);
  Serial.print("\t");
  Serial.println(swingup_state);


  if (digitalRead(PIN_SW_1) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }

  if (digitalRead(PIN_SW_2) == HIGH) {
    if (swingup_state == END) {
      swingup_state = START;
    }
  } else {
    swingup_state = END;
  }
  
  delay(20);
  digitalWrite(PIN_TIMER_2, timer2_pin_state ^= 1);
}
