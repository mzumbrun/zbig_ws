// Dev code for both left and right arduino nano iot for bigbot
// 01/12/2025 - removed mqtt and going stright to python on Upc
// 01/12/2025 - re-enabled mqtt and went to arduPID
// 01/02/2025 - removed mqtt and wifi in pm
// 01/02/2025 - test to see if ADC works better than PPM
// 01/01/2025 - initial mqtt code for bigbot
// 12/19/2024 - base code prior to MQTT
// 11/24/2024 - removing PID control from bigbot_control
// 11/21/2024 - on going pid iterations, speed control
// 11/20/2024 - removed softserial, removed string to end with #
// 09/16/2024 - enabled PID and encoder
// 09/05/2024 - changed serial read from ROS to read entire string
// 08/21/2024 - initial code

#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"
#include <ArduPID.h>

#define motor_pwm_pin 9    // ppm control signal to esc
#define motor_rev_pin 11   // low is reverse (3.3V per VESC)
#define motor_select 6     // connect to dc_high for right motor, dc_low for left motor
#define dc_high 5          // driven high - right
#define dc_low 4           // driven low - left
#define encoder_counter 2  // Interrupt
#define H2 3
#define H3 7


// WiFi network details
char ssid[] = SECRET_SSID;
char password[] = SECRET_PASS;

// MQTT server details
const char mqtt_server[] = "test.mosquitto.org";
const int mqtt_port = 1883;
String PIDmessage;
String messageTopic;

WiFiClient nanoClient;
MqttClient mqttClient(nanoClient);

// Encoders
// volatile bool state = false;
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
//double max_pos_speed = 1500.;  //  value from CALIBRATION to max rad/s from ROS
// double max_neg_speed = 1500.;
// double speed = 1500.;
// double max_rads_per_sec = 25.;  // 22 corresponds to that in ROS for 0.7 m/s
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor
double senddata = 0.;         // using double for ease of programming mqtt

ArduPID Motor;
//PID Motor(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd, DIRECT);

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
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
      // Serial.println("Connecting to WiFi.."); // comment when using ROS
  }
  // Serial.println("Connected to WiFi"); // comment when using ROS

 // mqttClient.connect(mqtt_server, mqtt_port);
  // if (!mqttClient.connect(mqtt_server, mqtt_port)) {
  //     Serial.print("MQTT connection failed! Error code = ");  // comment when using ROS
  //     Serial.println(mqttClient.connectError());              // comment when using ROS
  //   while (1)
  //     ;
  // }
  // Serial.println("Connected to MQTT"); // comment when using ROS

  mqttClient.onMessage(mqttCallback);
  mqttClient.subscribe("PID/senddata");

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
    // max_pos_speed = 1900;  // corresponds to max rad/s for RIGHT motor to match max provided by ROS
    // max_neg_speed = 1000;
    mqttClient.subscribe("PID/encoderR");
    mqttClient.subscribe("PID/clicksR");
    mqttClient.subscribe("PID/cmd_velR");
    mqttClient.subscribe("PID/actual_velR");
    mqttClient.subscribe("PID/KpR");
    mqttClient.subscribe("PID/KiR");
    mqttClient.subscribe("PID/KdR");
    Kp = KpR;
    Ki = KiR;
    Kd = KdR;
    Motor.setCoefficients(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    // max_pos_speed = 1900;
    // max_neg_speed = 1000;
    mqttClient.subscribe("PID/encoderL");
    mqttClient.subscribe("PID/clicksL");
    mqttClient.subscribe("PID/cmd_velL");
    mqttClient.subscribe("PID/actual_velL");

    mqttClient.subscribe("PID/KpL");
    mqttClient.subscribe("PID/KiL");
    mqttClient.subscribe("PID/KdL");
    Kp = KpL;
    Ki = KiL;
    Kd = KdL;
    //Motor.SetTunings(Kp, Ki, Kd);
    Motor.setCoefficients(Kp, Ki, Kd);
  }
}

void loop() {
  mqttClient.poll();
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
    // noInterrupts();
    // if (state) {
    //   encoder_count_++;
    //   state = false;
    // }
    // interrupts();
  }
  //**********************comment out line below for no PID
  //Motor.compute();
  //*************************
  // Encoder
  unsigned long current_millis = millis();
  real_interval = current_millis - last_millis;
  if (real_interval >= interval) {
    last_millis = current_millis;
    wheel_meas_vel = (1000. / real_interval) * (encoder_count_) * (60.0 / clicks_per_rev) * 0.10472;  //  rads/sec
    wheel_meas_vel = wheel_meas_vel / 20.;                                                                    // include gear ratio

    if (wheel_cmd_vel == 0.0) {  // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
    }

    if ('r' == wheel_side[0]) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel, 2) + ",";

    } else if ('l' == wheel_side[0]) {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel, 2) + ",";
    }
     Serial.println(encoder_read);  // send measured velocity back to ROS

    //***************************************************** */
    // to ignore PID, send to motors same cmd received from ROS2
    wheel_cmd = wheel_cmd_vel;             // wheel_cmd_vel is rad/
    wheel_cmd = (wheel_cmd / 10.) * 255.;  // scale
                                           // above only if ignoring PID

    // ***********  below for debug only *******************
    //  Serial.print(wheel_cmd_vel);          // rads/sec
    // Serial.print(",");
    //  Serial.print(wheel_meas_vel);  // rads/sec
    //  Serial.print(",");
    //  Serial.println(wheel_cmd);  // 0-255

    // Serial.print(",");
    // Serial.println(encoder_count_);
   // writeEncoderCount();
    if (senddata == 1.0) {
      writeEncoderCount();
      senddata = 0.;
    }
    // ********* above for debug only ***************
    encoder_count_ = 0;
  };
}
  analogWrite(motor_pwm_pin, wheel_cmd)

// New pulse from  Encoder
void EncoderCallback() {
  //state = true;
  encoder_count_++;
}
void mqttCallback(int length) {
  char message[20];
  double k;
  messageTopic = mqttClient.messageTopic();
  if (messageTopic == "PID/KpR") {
    Kp = convertMessage();
  } else if (messageTopic == "PID/KiR") {
    Ki = convertMessage();
  } else if (messageTopic == "PID/KdR") {
    Kd = convertMessage();
  } else if (messageTopic == "PID/KpL") {
    Kp = convertMessage();
  } else if (messageTopic == "PID/KiL") {
    Ki = convertMessage();
  } else if (messageTopic == "PID/KdL") {
    Kd = convertMessage();
  } else if (messageTopic == "PID/clicksR") {
    clicks_per_rev = convertMessage();
  } else if (messageTopic == "PID/clicksL") {
    clicks_per_rev = convertMessage();
  } else if (messageTopic == "PID/senddata") {
    senddata = convertMessage();
  }
  delay(100);
  Motor.setCoefficients(Kp, Ki, Kd);
}

double convertMessage() {
  // Serial.println(messageTopic);
  while (mqttClient.available()) {
    char inChar = ((char)mqttClient.read());
    PIDmessage += inChar;
  }
  double k = PIDmessage.toDouble();
  PIDmessage = "";
  return k;
}

void writeEncoderCount() {
  char message[20];
  if ('r' == wheel_side[0]) {
    dtostrf(encoder_count_, 6, 2, message);
    mqttClient.beginMessage("PID/encoderR");
    mqttClient.print(message);
    mqttClient.endMessage();

    dtostrf(clicks_per_rev, 6, 0, message);
    mqttClient.beginMessage("PID/clicksR");
    mqttClient.print(message);
    mqttClient.endMessage();

    dtostrf(wheel_cmd_vel, 6, 2, message);  // rad/s command from ROS
    mqttClient.beginMessage("PID/cmd_velR");
    mqttClient.print(message);
    mqttClient.endMessage();

    dtostrf(wheel_meas_vel, 6, 2, message);  // rad/s measured from encoder values
    mqttClient.beginMessage("PID/actual_velR");
    mqttClient.print(message);
    mqttClient.endMessage();
  } else if ('l' == wheel_side[0]) {
    dtostrf(encoder_count_, 6, 2, message);
    mqttClient.beginMessage("PID/encoderL");
    mqttClient.print(message);
    mqttClient.endMessage();

    dtostrf(clicks_per_rev, 6, 0, message);
    mqttClient.beginMessage("PID/clicksL");
    mqttClient.print(message);
    mqttClient.endMessage();

    dtostrf(wheel_cmd_vel, 6, 2, message);  // rad/s command from ROS
    mqttClient.beginMessage("PID/cmd_velL");
    mqttClient.print(message);
    mqttClient.endMessage();

    dtostrf(wheel_meas_vel, 6, 2, message);  // rad/s measured from encoder values
    mqttClient.beginMessage("PID/actual_velL");
    mqttClient.print(message);
    mqttClient.endMessage();
  }
}

char* dtostrf(double val, signed char width, unsigned char prec, char* sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}