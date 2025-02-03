// POR code for both left and right arduinos for bigbot
// to do - remove dc_high pin, use input_pullup. not urgent
// 09/06/2024 - uses PID and encoder
// 09/05/2024 - changed serial read from ROS to read entire string
// 08/21/2024 - initial code


#include <PID_v2.h>
#include <Servo.h>

#define motor_ppm_pin 9    // ppm control signal to esc
#define motor_select 6     // connect to dc_high for right motor, dc_lowfor left motor
#define dc_high 5          // driven high - right
#define dc_low 4           // driven low - left
#define encoder_counter 2  // Interrupt

Servo bigbot_servo;

// Encoders
unsigned int encoder_count_ = 0;
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
String wheel_sign = "p";  // 'p' = positive, 'n' = negative
bool is_wheel_forward = true;
String value = "00.00";
String ROS_input = "rp00.00,lp00.00,";
String encoder_read = "rp00.00,";
char wheel_side[] = "right";
bool is_right = true;

// speed control
int max_pos_speed = 1500;  //  value from CALIBRATION to max rad/s from ROS
int max_neg_speed = 1500;
int max_rads_per_sec = 8;     // corresponds to that in ROS
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor
double Kp = 0.;               // orig 12.8
double Ki = 0.;               // orig 8.3
double Kd = 0.;               // orig 0.1
PID Motor(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd, DIRECT);

void setup() {

  Motor.SetMode(AUTOMATIC);

  bigbot_servo.attach(motor_ppm_pin);
  pinMode(motor_ppm_pin, OUTPUT);
  bigbot_servo.writeMicroseconds(1500);  // start the motor at 0 speed

  pinMode(dc_low, OUTPUT);
  pinMode(dc_high, OUTPUT);
  pinMode(motor_select, INPUT);
  pinMode(encoder_counter, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, RISING);
  digitalWrite(dc_low, LOW);
  digitalWrite(dc_high, HIGH);

  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    is_right = true;
    max_pos_speed = 2000;  // corresponds to max rad/s for RIGHT motor to match max provided by ROS
    max_neg_speed = 1200;
    Kp = 15000.0;
    Ki = 800.0;
    Kd = 0.1;
    Motor.SetTunings(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    max_pos_speed = 2000;
    max_neg_speed = 1200;
    Kp = 15000.0;
    Ki = 800.0;
    Kd = 0.1;
    Motor.SetTunings(Kp, Ki, Kd);
  }
  Serial.begin(115200);
  bigbot_servo.writeMicroseconds(1500); // start with motors off
}

void loop() {

  // format from ros: "rdxx.xx,ldxx.xx,"
  if (Serial.available() >0 ) {
    ROS_input = Serial.readStringUntil('X');  // \0 is null character  \n is new line
    if (wheel_side[0] == 'r') {                // if true, then get its direction and speed
      is_right = true;
      wheel_sign = ROS_input[1];
      value[0] = ROS_input[2];
      value[1] = ROS_input[3];
      value[2] = ROS_input[4];
      value[3] = ROS_input[5];
      value[4] = ROS_input[6];
      value[5] = '\0';
    }
    if (wheel_side[0] == 'l') {  // if true, then get its direction and speed
      wheel_sign = ROS_input[9];
      value[0] = ROS_input[10];
      value[1] = ROS_input[11];
      value[2] = ROS_input[12];
      value[3] = ROS_input[13];
      value[4] = ROS_input[14];
      value[5] = '\0';
    }
    wheel_cmd_vel = value.toFloat();
    if (wheel_sign == "p") {
      is_wheel_forward = true;
    } else {
      is_wheel_forward = false;
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    last_millis = current_millis;
    wheel_meas_vel = (10 * encoder_count_ * (60.0 / 175.)) * 0.10472;  //  rads/sec
   // Serial.println("count/s  " + String(encoder_count_));
   // Serial.println("rad/s   " + String(wheel_meas_vel));
   // Serial.println(" ");
    encoder_count_ = 0;
    // Motor.Compute();  // output is wheel_cmd in rad/s

    if (wheel_cmd_vel == 0.0) {  // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
    }
// *** UNTIL ENCODER CAN WORK, SEND BACK TO ROS SAME AS INPUT. ULTIMATE CHANGE TO wheel_meas_vel
   wheel_meas_vel=wheel_cmd_vel; // forcing measured to be same as setpoint
   wheel_cmd = wheel_cmd_vel; // passing input to motors and back to ROS
// ***

    if (is_right) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel, 2) + ",";
    } else {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel, 2) + ",";
    }
    Serial.println(encoder_read);

    //*****************************************************
    if (is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd, 0, 10, 1500, max_pos_speed));
      // wheel_sign = "p";
    }
    if (!is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd, 0, 10, 1500, max_neg_speed));
      // wheel_sign = "n";
    }
  }
}


// New pulse from Left Wheel Encoder
void EncoderCallback() {
  encoder_count_++;
}
