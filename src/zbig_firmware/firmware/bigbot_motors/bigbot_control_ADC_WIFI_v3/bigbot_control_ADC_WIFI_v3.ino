// Dev code for both left and right arduino nano iot for bigbot
// 01/24/2025 going back to ADC,


#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include "bigbot_firmware_parameters.h"
#include <ArduPID.h>

#define motor_true_adc_pin A0  //
#define motor_rev_pin 11       // low is reverse (3.3V per VESC)
#define motor_select 6         // connect to dc_high for right motor, dc_low for left motor
#define dc_high 5              // driven high - right
#define dc_low 4               // driven low - left
#define encoder_counter 2      // Interrupt for hall sensor in use

// WiFi network details
char ssid[] = SECRET_SSID;
char password[] = SECRET_PASS;
int status = WL_IDLE_STATUS;
unsigned int leftPort = 2390;  // local port to listen
unsigned int rightPort = 2490;
const int PACKET_SIZE = 10;
char packetBuffer[PACKET_SIZE];    //buffer to hold packet RECEIVED
char message_string[PACKET_SIZE];  // message packet to SEND
WiFiClient nanoClient;
WiFiUDP Udp;

// Encoders
unsigned long encoder_count_ = 0;
unsigned int clicks_per_rev = 24.;  // encoder-plate-v1 is 24
unsigned long last_millis = 0;
const unsigned long interval = 333;
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
//PID Motor(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd, DIRECT);

void setup() {

  pinMode(motor_true_adc_pin, OUTPUT);
  pinMode(motor_rev_pin, OUTPUT);
  pinMode(dc_low, OUTPUT);
  pinMode(dc_high, OUTPUT);
  digitalWrite(dc_low, LOW);
  digitalWrite(dc_high, HIGH);
  pinMode(motor_select, INPUT);

  pinMode(encoder_counter, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, FALLING);

  analogWrite(motor_rev_pin, LOW);     // default backwards
  analogWrite(motor_true_adc_pin, 0);  // motors off at start

  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");  // comment when using ROS
  }
  //  Serial.println("Connected to WiFi"); // comment when using ROS
  printWifiStatus();

  //PID
  Motor.begin(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd);
  Motor.setSampleTime(interval);

                                  // Motor.setWindUpLimits(-0., 20.0);  // Left -1,3 ; best -.5, .5
  Motor.start();
  //****

  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    Udp.begin(rightPort);
    is_right = true;
    Kp = KpR;
    Ki = KiR;
    Kd = KdR;
    Motor.setOutputLimits(16, 80);  // left 20,80, right 20,80 for now
    Motor.setCoefficients(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    Udp.begin(leftPort);
    is_right = false;
    Kp = KpL;
    Ki = KiL;
    Kd = KdL;
    Motor.setOutputLimits(20, 80);  // left 20,80, right 20,80 for now
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
        analogWrite(motor_rev_pin, 255);  // 3.3 volts
      }
    } else if (chr == 'n') {
      if (is_wheel_cmd) {
        wheel_sign = "n";
        is_wheel_forward = false;
        analogWrite(motor_rev_pin, 0);  // 0 volts
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

  // ************ start of speed measurement
  unsigned long current_millis = millis();
  real_interval = current_millis - last_millis;
  if (real_interval >= interval) {
    last_millis = current_millis;
    wheel_meas_vel = (1000. / real_interval) * (encoder_count_) * (60.0 / clicks_per_rev) * 0.10472;  //  rads/sec

    if ('r' == wheel_side[0]) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel, 2) + ",";

    } else if ('l' == wheel_side[0]) {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel, 2) + ",";
    }
    Serial.println(encoder_read);  // send measured velocity back to ROS

    Motor.compute();
    //*************************************************************************edit for PID ***************************************** */
    // to ignore PID, send to motors same cmd received from ROS2
    //wheel_cmd = wheel_cmd_vel;  // wheel_cmd_vel is rad/s // ************** comment out if using PID
    // above only if ignoring PID
    //*****************************************************
    encoder_count_ = 0;
    // }

    // *********end of speed measurement
    //
    if (wheel_cmd_vel == 0.0) {  // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
      encoder_count_ = 0;
    }
    // send speed to motors
    analogWrite(motor_true_adc_pin, (int)wheel_cmd);
  }
  tunePID();
}

// New pulse from  Encoder
void EncoderCallback() {
  //state = true;
  encoder_count_++;
}
char* dtostrf(double val, signed char width, unsigned char prec, char* sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void tunePID() {
  // format from python "axxx.xxx," where a is p, i, or d and xxx.xxx is corresponding value to use
  int packetSize = Udp.parsePacket();
  // only proceeds if data sent
  if (packetSize > 0) {
    int len = Udp.read(packetBuffer, 255);
    // if (len > 0) {
    //   packetBuffer[len] = 0;
    // }
    String myPID = String(packetBuffer).substring(1, len);

    if (packetBuffer[0] == 'p') {
      Kp = myPID.toDouble();
    } else if (packetBuffer[0] == 'i') {
      Ki = myPID.toDouble();
    } else if (packetBuffer[0] == 'd') {
      Kd = myPID.toDouble();
    } else if (packetBuffer[0] == 's') {
      sendData();
    }
    Motor.setCoefficients(Kp, Ki, Kd);
  }
}

void sendData() {

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

  //dtostrf(encoder_count_, PACKET_SIZE, 0, message_string);  // was wheel_cmd_vel & 3 decimal places
  dtostrf(wheel_cmd_vel, PACKET_SIZE, 2, message_string);  // was wheel_cmd_vel & 3 decimal places
  Udp.write(message_string, PACKET_SIZE);

  dtostrf(wheel_meas_vel, PACKET_SIZE, 2, message_string);
  Udp.write(message_string, PACKET_SIZE);

  dtostrf((int)wheel_cmd, PACKET_SIZE, 0, message_string);
  Udp.write(message_string, PACKET_SIZE);

  dtostrf(Kp, PACKET_SIZE, 3, message_string);
  Udp.write(message_string, PACKET_SIZE);

  dtostrf(Ki, PACKET_SIZE, 3, message_string);
  Udp.write(message_string, PACKET_SIZE);

  dtostrf(Kd, PACKET_SIZE, 3, message_string);
  Udp.write(message_string, PACKET_SIZE);

  Udp.endPacket();
}