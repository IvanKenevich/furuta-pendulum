#include <Encoder.h>

#define rad2deg (180.0 / M_PI)
#define deg2rad (M_PI / 180.0)

//#define LED_BUILTIN 13
#define PIN_ENC1_A   2 // encoder pins for the motor
#define PIN_ENC1_B   3
#define PIN_ENC2_A   2 // encoder pins for the free arm
#define PIN_ENC2_B   3
#define PIN_PWM1    29 // PWM pins for motor driver
#define PIN_PWM2    30 
#define PIN_TIMER_1   32 // indicator timer pin
#define PIN_TIMER_2   31
#define PIN_SW_1    33 // pins connected to switches
#define PIN_SW_2    39
#define PIN_SW_3    15

#define TIMER_CYCLE_MICROS 1 * 100
const float Ts = 1e-6 * TIMER_CYCLE_MICROS;
IntervalTimer t1;
const float input_freq = 4, // Hz
            input_amp  = 5, // V
            supply_voltage = 12; // V
const int pos_buff_size = 50; // samples
unsigned long i = 0;

Encoder enc1(PIN_ENC1_A, PIN_ENC1_B),
        enc2(PIN_ENC2_A, PIN_ENC2_B);
volatile long enc1_pos_raw, enc2_pos_raw;
volatile float enc1_pos, enc2_pos, enc2_speed;
float enc2_pos_buff[pos_buff_size] = {0};

volatile char timer1_pin_state = 0, timer2_pin_state = 0;
volatile float pwm_out = 0, pwm_out_copy = 0, error = 0, setpoint = 0, integral = 0;
volatile int motor_enabled = false, integral_enabled = false;


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
  enc1_pos = 360.0 * enc1_pos_raw / 8192.0;  // degrees
  enc2_pos = 360.0 * enc2_pos_raw / 1632.67;

  for (int j = pos_buff_size - 1; j > 0; j--) {
    enc2_pos_buff[j] = enc2_pos_buff[j - 1];
  }
  enc2_pos_buff[0] = enc2_pos;
  enc2_speed = (enc2_pos_buff[0] - enc2_pos_buff[pos_buff_size - 1]) / ((pos_buff_size - 1) * Ts);
  

//  setpoint = 45.0 * sin(input_freq * 2 * M_PI * Ts * i);
//  error = (setpoint - enc2_pos);
//  if (integral_enabled) {
//    integral += error;
//  }
//  else {
//    integral = 0;
//  }
//  pwm_out = error * 1.5 + integral * 0.005;

  setpoint = input_amp * sin(input_freq * 2 * M_PI * Ts * i);
  pwm_out = 100.0 * setpoint / supply_voltage;

  if (motor_enabled) {
    write_pwm(pwm_out);
  } else {
    write_pwm(0);
  }
  ++i;

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
  Serial.println(enc2_speed);


  if (digitalRead(PIN_SW_1) == HIGH) {
    motor_enabled = true;
  } else {
    motor_enabled = false;
  }

  if (digitalRead(PIN_SW_2) == HIGH) {
    integral_enabled = true;
  } else {
    integral_enabled = false;
  }
  
  delay(20);
  digitalWrite(PIN_TIMER_2, timer2_pin_state ^= 1);
}
