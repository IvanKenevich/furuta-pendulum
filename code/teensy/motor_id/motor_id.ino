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
  Motor variables
*/
#define PIN_PWM1    29 // PWM pins for motor driver
#define PIN_PWM2    30 
volatile float pwm_out = 0, pwm_out_copy = 0;

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
#define PIN_SW_2    39
#define PIN_SW_3    15

#define TIMER_CYCLE_MICROS 1 * 100
const float Ts = 1e-6 * TIMER_CYCLE_MICROS;
IntervalTimer t1;

elapsedMicros t_us; // teensyduino variable type that increments by itself
const float input_freq = 4, // Hz
            input_amp  = 5, // V
            supply_voltage = 12; // V
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
  enc1_pos = 360.0 * enc1_pos_raw / 1632.67;  // degrees

  // shift over position buffer and add the new measurement
  for (int j = ENC1_POS_BUFF - 1; j > 0; j--) {
    enc1_pos_buff[j] = enc1_pos_buff[j - 1];
  }
  enc1_pos_buff[0] = enc1_pos;

  // calculate speed as a secant line to position
  enc1_speed = (enc1_pos_buff[0] - enc1_pos_buff[ENC1_POS_BUFF - 1]) / ((ENC1_POS_BUFF - 1) * Ts);

  setpoint = input_amp * sin(input_freq * 2 * M_PI * t_us * 1e-6);
  pwm_out = 100.0 * setpoint / supply_voltage;

  if (motor_enabled) {
    write_pwm(pwm_out);
  } else {
    write_pwm(0);
  }

  digitalWrite(PIN_TIMER_1, timer1_pin_state ^= 1);
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
  //  noInterrupts();
  //  pwm_out_copy = pwm_out;
  //  interrupts();
  //
  //
  Serial.print(setpoint);
  Serial.print("\t");
//  Serial.print(enc2_pos);
//  Serial.print("\t");
  Serial.println(enc1_speed);


  if (digitalRead(PIN_SW_1) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }
  
  delay(20);
  digitalWrite(PIN_TIMER_2, timer2_pin_state ^= 1);
}
