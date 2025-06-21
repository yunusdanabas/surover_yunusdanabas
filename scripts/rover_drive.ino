#include <Arduino.h>
#define ENCODER_OPTIMIZE_INTERRUPTS // Don't use attachInterrupt()
#include <Encoder.h>

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
sensor_msgs::Joy joy_msgs;
ros::NodeHandle nh;

// Motor Struct For Rover Drive
struct Motor {
  int motorPWM = 0;
  int motorENA = 0;
  float RPM = 0;
  float wheel_rotation = 0; // Check with bigger wheel
  
  int pinA;
  int pinB;
  Encoder encoder;
  int pinPWM;
  int pinENA;
  int pinCurrentSense;
  
  // Constructor
  Motor(int pinA, int pinB, int pinPWM, int pinENA, int pinCurrentSense) :
    pinA(pinA), pinB(pinB), encoder(pinA, pinB), pinPWM(pinPWM), pinENA(pinENA), pinCurrentSense(pinCurrentSense)
  {
    pinMode(pinPWM, OUTPUT);
    pinMode(pinENA, OUTPUT);
    pinMode(pinCurrentSense, INPUT);
  }
  
  long getEncoderCount() {
    return encoder.read();
  }
  
  void giveSignals() {
    analogWrite(pinPWM, motorPWM);
    digitalWrite(pinENA, motorENA);
  }
  
  float readCurrent() {
    int sensorValue = analogRead(pinCurrentSense);
    float voltage = sensorValue * (5.0 / 1023.0); // Convert to voltage (assuming 5V reference)
    float current = voltage / 0.185; // Convert to current (assuming 185 mV/A)
    return current;
  }
};

// Function Declarations
void parseData(const sensor_msgs::Joy &);
void rotVelocity(Motor &motor); // Motor A
// void controlLaw();

// Encoder count values
long encoderCount = 0;
long encoderLastCount = 0;
// Velocity variables
long currentMicro = 0;
long previousMicro = 0;
float deltaTime = 0;

Motor leftMotor(2, 3, 10, 9, A0); // Assume current sense pin is A0
Motor rightMotor(4, 5, 11, 8, A1); // Assume current sense pin is A1

// Drive Variables
float x = 0;
float y = 0;
float z = 0;
float sticktoPWM = 127;
float yPWM = 0;
float xPWM = 0;
float zPWM = 0;
void joystickDIR();
void checkPWM();
void NewDrive(Motor &x, Motor &y);

ros::Subscriber<sensor_msgs::Joy> sub("joy", parseData);
void setup() {
  nh.initNode();
  nh.subscribe(sub);

  leftMotor.motorENA = 1;
  rightMotor.motorENA = 1;
}

void loop() {
  nh.spinOnce();
  delay(1);
}

void rotVelocity(Motor &motor) {
  encoderCount = motor.getEncoderCount();
  motor.wheel_rotation = encoderCount / (float(6000));
  // Same times for diff motors?
  // Change to millis, memory problems
  currentMicro = micros();
  deltaTime = ((float)(currentMicro - previousMicro)) / 1.0e6;
  motor.RPM = (float(60) * (encoderCount - encoderLastCount) / float(6000)) / deltaTime;
  encoderLastCount = encoderCount;
  previousMicro = currentMicro;
}

void joystickDIR() {
  xPWM = sticktoPWM * x;
  yPWM = sticktoPWM * y;
  zPWM = sticktoPWM * z;
}

void checkPWM() {
  if (leftMotor.pinPWM <= 2.55) {
    leftMotor.pinPWM = 2.55;
  }
  if (rightMotor.pinPWM <= 2.55) {
    rightMotor.pinPWM = 2.55;
  }
  if (leftMotor.pinPWM > 247.45) {
    leftMotor.pinPWM = 247.45;
  }
  if (rightMotor.pinPWM > 247.45) {
    rightMotor.pinPWM = 247.45;
  }
}

void NewDrive(Motor &left, Motor &right) {
  int valLeft = yPWM - zPWM / 2;
  if (valLeft > 0) {
    left.motorPWM = map(abs(valLeft), 0, 382.5, 127.5, 247.55); // Forward
  } else if (valLeft < 0) {
    left.motorPWM = map(abs(valLeft), 0, 382.5, 127.5, 2.55); // Reverse
  }

  int valRight = yPWM + zPWM / 2;
  if (valRight > 0) {
    right.motorPWM = map(abs(valRight), 0, 382.5, 127.5, 247.55); // Forward
  } else if (valRight < 0) {
    right.motorPWM = map(abs(valRight), 0, 382.5, 127.5, 2.55); // Reverse
  }
}

void parseData(const sensor_msgs::Joy &message) {
  joy_msgs = message;
  x = joy_msgs.axes[0];
  y = joy_msgs.axes[1];
  z = joy_msgs.axes[2];
  joystickDIR();
  NewDrive(leftMotor, rightMotor);
  checkPWM();
  
  String s = String(leftMotor.motorPWM) + " " + String(rightMotor.motorPWM) + " ";
  s += "Left Current: " + String(leftMotor.readCurrent()) + "A ";
  s += "Right Current: " + String(rightMotor.readCurrent()) + "A\n";
  nh.loginfo(s.c_str());

  leftMotor.giveSignals();
  rightMotor.giveSignals();
}
