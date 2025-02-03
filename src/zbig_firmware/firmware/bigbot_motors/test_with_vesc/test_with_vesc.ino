/*
9/1/2024 - use this with vesc program to calibrate ppm to rad/s
         - connect arduino to ubuntu pc, connect vesc to wsl pc with vesc program
         - enter ppm and use vesc program with real time data on for erpm, voltage, etc. Record data.
7/29/24 - debug arduino to vesc
*/
#include <Servo.h>

Servo m1_servo;
String inString = "";  // string to hold input from keyboard
#define m1_ppm_pin 9
#define encoder_interrupt_1 2
double speed = 0.0;
double rpm = 0.0;  // Measured from motor encoders, rad/s
volatile bool state = false;
unsigned int encoder_counter1 = 0;
unsigned long last_millis = 0;
const unsigned long interval = 100;

void setup() {
  pinMode(encoder_interrupt_1, INPUT_PULLUP);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_1), Encoder1Callback, LOW);
  while (!Serial) {
  }
  m1_servo.attach(m1_ppm_pin);
  pinMode(m1_ppm_pin, OUTPUT);
}

void loop() {
  // Read serial input:
  ///*
  if (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if a newline, print the string, then the string's value:
    if (inChar == '\n') {
      Serial.print("ppm speed: ");
      speed = inString.toFloat();
      //speed = inString.toInt();
      Serial.println(speed);
      inString = "";
      Serial.println(speed);
      m1_servo.writeMicroseconds(speed);
    }
  }

  //noInterrupts();
  //if (state) {
  //  encoder_counter1++;
  //  state = false;
  //}
  //interrupts();

  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    last_millis = current_millis;    
    rpm = (10. * encoder_counter1* (60.0 / 17500.));  //  
    Serial.println("encoder count =     " + String(encoder_counter1) + "       rpm = " + String(rpm));
    Serial.println(" ");
    encoder_counter1 = 0;
  }
}
void Encoder1Callback() {
  //state = true;
  encoder_counter1++;
}