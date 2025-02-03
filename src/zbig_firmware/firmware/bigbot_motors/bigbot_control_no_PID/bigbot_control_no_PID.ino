// POR code for both left and right arduinos for bigbot
// to do - remove dc_high pin, use input_pullup. not urgent
// 12/19/2024 - base code prior to MQTT
// 11/24/2024 - removing PID control from bigbot_control
// 11/21/2024 - on going pid iterations, speed control
// 11/20/2024 - removed softserial, removed string to end with #
// 09/16/2024 - enabled PID and encoder
// 09/05/2024 - changed serial read from ROS to read entire string
// 08/21/2024 - initial code


#include <PID_v1.h>
#include <Servo.h>

#define motor_ppm_pin 9    // ppm control signal to esc
#define motor_select 6     // connect to dc_high for right motor, dc_low for left motor
#define dc_high 5          // driven high - right
#define dc_low 4           // driven low - left
#define encoder_counter 2  // Interrupt
#define H2 3
#define H3 7

Servo bigbot_servo;

// Encoders
volatile bool state = false;
unsigned long encoder_count_ = 0;
unsigned long last_millis = 0;
const unsigned long interval = 500;
unsigned long real_interval = 0;
char len = "1";

// Interpret Serial Messages
String wheel_sign = "p";  // 'p' = positive, 'n' = negative
bool is_wheel_cmd = false;
bool is_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
String encoder_read = "rp00.00,";
char wheel_side[] ="right";
bool is_right = true;
char chr= "x";

// speed control
double max_pos_speed = 1500.;  //  value from CALIBRATION to max rad/s from ROS
double max_neg_speed = 1500.;
double speed =1500.;
double max_rads_per_sec = 25.;     // 22 corresponds to that in ROS for 0.7 m/s
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor
double Kp = 30.;               // orig 12.8
double Ki = 0.6;               // orig 8.3
double Kd = 5.;               // orig 0.1
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
  pinMode(H2, INPUT_PULLUP);
  pinMode(H3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, FALLING);
  digitalWrite(dc_low, LOW);
  digitalWrite(dc_high, HIGH);

  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    is_right = true;
    max_pos_speed = 2200;  // corresponds to max rad/s for RIGHT motor to match max provided by ROS
    max_neg_speed = 1000;
    Kp = 30.;  // was 10.
    Ki = 0.6;   // was 0.8
    Kd = 5.;  // was 0.1
    Motor.SetTunings(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    max_pos_speed = 2200;
    max_neg_speed = 1000;
    Kp = 30.;
    Ki = 0.6;
    Kd = 5.;
    Motor.SetTunings(Kp, Ki, Kd);
  }

  Serial.begin(115200);
  bigbot_servo.writeMicroseconds(1500);  // start with motors at zero speed
}

void loop() {

  // format from ros: "rdxx.xx,ldxx.xx,"
  if (Serial.available() > 0) {
    chr = Serial.read();
 // \0 is null character  \n is new line
    if (chr == wheel_side[0]) {               
      is_wheel_cmd = true;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if (chr == 'p') {
     if (is_wheel_cmd) {
      wheel_sign ="p";
      is_wheel_forward = true;
     }
    }
    else if (chr == 'n') {
     if (is_wheel_cmd) {
      wheel_sign ="n";
      is_wheel_forward = false;
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
    noInterrupts();
    if (state) {
      encoder_count_++;
      state = false;
    }
    interrupts();
  }

  // Encoder
  unsigned long current_millis = millis();
  real_interval = current_millis - last_millis;
  if (real_interval >= interval) {
    last_millis = current_millis;
  
    wheel_meas_vel = (1000/real_interval)* encoder_count_ * (60.0 / 35.) * 0.10472;  //  rads/sec
    encoder_count_=0;
    // Motor.Compute();  // output is wheel_cmd 0-255


    if (wheel_cmd_vel == 0.0) {  // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
    }


// following commands not necessary if using PID to zero in on matched speed with ROS
 //  wheel_cmd = constrain(wheel_cmd, 0, 255);
// above not necessary if using PID

// send back to ROS what was sent to Arduino -- EFFECTIVELY not using encoder
wheel_meas_vel = wheel_cmd_vel;

    if ('r'== wheel_side[0]) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel, 2) + ",";

    } else if ('l' == wheel_side[0]) {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel, 2) + ",";

    }
    Serial.println(encoder_read);

// wheel_cmd is between 0-255 if output from PID
// wheel_cmd = wheel_cmd: // if using wheel_cmd then using PID
wheel_cmd = constrain(wheel_cmd, 0, max_rads_per_sec); // if using wheel_cmd_vel then sending command from ROS directly to motor


    //*****************************************************
    if (wheel_sign == "p") {
      speed = 1500. + (wheel_cmd / max_rads_per_sec)* (max_pos_speed -1500.);
      bigbot_servo.writeMicroseconds(int(speed));
      // wheel_sign = "p";
    }
    if (wheel_sign == "n") {
      speed = 1500. + (wheel_cmd / max_rads_per_sec)*(max_neg_speed-1500.);
      bigbot_servo.writeMicroseconds(int(speed));
      // wheel_sign = "n";
    }
  }
}

// New pulse from  Encoder
void EncoderCallback() {
  state = true;
}
