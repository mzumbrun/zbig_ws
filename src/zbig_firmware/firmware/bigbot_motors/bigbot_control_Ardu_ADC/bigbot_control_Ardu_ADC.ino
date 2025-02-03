// Dev code for both left and right arduino nano iot for bigbot
// 01/06/2025 - moved to ArduPID for PID control given its bias ability
// 01/03/2025 - bare ADC code

#include "arduino_secrets.h"
#include "ArduPID.h"

#define motor_pwm_pin 9    // ppm control signal to esc
#define motor_rev_pin 11   // low is reverse (3.3V per VESC)
#define motor_select 6     // connect to dc_high for right motor, dc_low for left motor
#define dc_high 5          // driven high - right
#define dc_low 4           // driven low - left
#define encoder_counter 2  // Interrupt
#define H2 3
#define H3 7

// Encoders
volatile bool state = false;
unsigned long encoder_count_ = 0;
unsigned int clicks_per_rev = 4096;
unsigned long last_millis = 0;
const unsigned long interval = 100;
unsigned long real_interval = 0;

// Interpret Serial Messages
String wheel_sign = "p";  // 'p' = positive, 'n' = negative
bool is_wheel_cmd = false;
bool is_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
String encoder_read = "rp00.00,";
char wheel_side[] = "right";
bool is_right = true;

// speed control
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor

ArduPID Motor;

// PID Motor(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd, DIRECT);

void setup() {

  pinMode(dc_low, OUTPUT);
  pinMode(dc_high, OUTPUT);
  pinMode(motor_pwm_pin, OUTPUT);
  pinMode(motor_rev_pin, OUTPUT);
  pinMode(motor_select, INPUT);
  pinMode(encoder_counter, INPUT_PULLUP);
  pinMode(H2, INPUT_PULLUP);
  pinMode(H3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, RISING);
  digitalWrite(dc_low, LOW);
  digitalWrite(dc_high, HIGH);
  analogWrite(motor_rev_pin, LOW);  // default backwards
  analogWrite(motor_pwm_pin, 0);    // motors off at start

  Serial.begin(115200);
  //Motor.SetMode(AUTOMATIC);
  Motor.begin(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd);
  //Motor.setOutputLimits(0, 10);
  //Motor.setBias(58);  // best so far is L at 58
  Motor.setSampleTime(10);
  //Motor.setWindUpLimits(-10, 100);  // Left -1,3
  
  Motor.start();

  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    is_right = true;
    Kp = KpR;
    Ki = KiR;
    Kd = KdR;
    //Motor.SetTunings(Kp, Ki, Kd);
    Motor.setCoefficients(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    Kp = KpL;
    Ki = KiL;
    Kd = KdL;
    //Motor.SetTunings(Kp, Ki, Kd);
    Motor.setCoefficients(Kp, Ki, Kd);
  }
}

void loop() {
  // format from ros: "rdxx.xx,ldxx.xx,"
  if (Serial.available() > 0) {
    char chr = Serial.read();
    // \0 is null character  \n is new line
    if (chr == wheel_side[0]) {
      is_wheel_cmd = true;
      value_idx = 0;
      is_cmd_complete = false;
    } else if (chr == 'p') {
      if (is_wheel_cmd) {
        wheel_sign = "p";
        is_wheel_forward = true;
        analogWrite(motor_rev_pin, 168);  // 3.3 volts
      }
    } else if (chr == 'n') {
      if (is_wheel_cmd) {
        wheel_sign = "n";
        is_wheel_forward = false;
        analogWrite(motor_rev_pin, LOW);
      }
    }
    // Separator
    else if (chr == ',') {
      if (is_wheel_cmd) {
        wheel_cmd_vel = atof(value);  // rad/s from ROS to command the wheels
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
      is_wheel_cmd = false;
    }
    // Command Value
    else {
      if (value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }
  //Serial.println(state);
  // noInterrupts();
  // if (state) {
  //   encoder_count_++;
  //   //.println(encoder_count_);
  //   state = false;
  // }
  // interrupts();
  //
  //**********************comment out line below for no PID
  //Motor.compute();
  //*************************

  // Encoder
  unsigned long current_millis = millis();
  real_interval = current_millis - last_millis;
  if (real_interval >= interval) {
    last_millis = current_millis;
    wheel_meas_vel = (1000. / real_interval) * (encoder_count_-1500.) * (60.0 / clicks_per_rev) * 0.10472;  //  rads/sec
    wheel_meas_vel = wheel_meas_vel / 20.;      // include gear ratio

   // Motor.compute();  // output is wheel_cmd

    if (wheel_cmd_vel == 0.0) {  // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
    }

    if ('r' == wheel_side[0]) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel, 2) + ",";

    } else if ('l' == wheel_side[0]) {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel, 2) + ",";
    }
   // Serial.println(encoder_read);  // send measured velocity back to ROS

    //***************************************************** */
    // to ignore PID, send to motors same cmd received from ROS2
    wheel_cmd = wheel_cmd_vel;  // wheel_cmd_vel is rad/
    wheel_cmd = (wheel_cmd / 10.) * 255.;  // scale 
                                // above only if ignoring PID
    //*****************************************************

 

    // ***********  below for debug only *******************
   Serial.print(wheel_cmd_vel);          // rads/sec
  Serial.print(",");
   Serial.print(wheel_meas_vel);  // rads/sec
   Serial.print(",");
   Serial.println(wheel_cmd);  // 0-255
    // Serial.print(",");
    // Serial.println(encoder_count_);
    // ********* above for debug only ***************
    encoder_count_=0;
  }
  analogWrite(motor_pwm_pin, wheel_cmd);
}

// New pulse from  Encoder
void EncoderCallback() {
  state = true;
  encoder_count_++;
  //Serial.println(state);
}