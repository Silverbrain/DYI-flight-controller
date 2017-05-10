#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <PinChangeInt.h>

// MPU6050 instance
MPU6050 mpu;

// Servo instances for motors
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Motor pin definitions
const int motor1Pin = 9;
const int motor2Pin = 10;
const int motor3Pin = 11;
const int motor4Pin = 12;

// RC input pins
const int throttlePin = 2;
const int rollPin = 3;
const int pitchPin = 4;
const int yawPin = 5;

// RC input values
volatile int throttle = 0;
volatile int roll = 0;
volatile int pitch = 0;
volatile int yaw = 0;

// PID variables
float kp = 0.6;
float ki = 0.3;
float kd = 0.2;

float setPoint = 0;
float input, output;
float errSum, lastErr;

// Time variables
unsigned long lastTime;
unsigned long timeChange;

// Function to read RC inputs
void readRCInputs() {
  throttle = pulseIn(throttlePin, HIGH);
  roll = pulseIn(rollPin, HIGH);
  pitch = pulseIn(pitchPin, HIGH);
  yaw = pulseIn(yawPin, HIGH);
}

void setup() {
  // Initialize I2C communication
  Wire.begin();

  // Initialize the MPU6050
  mpu.initialize();

  // Initialize motor control pins
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);

  // Initialize RC input pins
  pinMode(throttlePin, INPUT);
  pinMode(rollPin, INPUT);
  pinMode(pitchPin, INPUT);
  pinMode(yawPin, INPUT);

  // Initialize motors to off
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  // Wait for ESCs to initialize
  delay(2000);

  // Initialize PID variables
  lastTime = millis();
  errSum = 0;
  lastErr = 0;
}

void loop() {
  // Get the current time
  unsigned long now = millis();
  timeChange = (now - lastTime);

  // Read RC inputs
  readRCInputs();

  // Read sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Use gyro data (gx, gy, gz) for PID input
  input = gx;

  // Calculate error
  float error = setPoint - input;
  errSum += (error * timeChange);
  float dErr = (error - lastErr) / timeChange;

  // Calculate PID output
  output = kp * error + ki * errSum + kd * dErr;

  // Adjust motor speeds based on PID output and RC inputs
  int motorSpeed1 = constrain(throttle + output + roll - pitch + yaw, 1000, 2000);
  int motorSpeed2 = constrain(throttle - output - roll - pitch - yaw, 1000, 2000);
  int motorSpeed3 = constrain(throttle + output - roll + pitch - yaw, 1000, 2000);
  int motorSpeed4 = constrain(throttle - output + roll + pitch + yaw, 1000, 2000);

  // Write speeds to motors
  motor1.writeMicroseconds(motorSpeed1);
  motor2.writeMicroseconds(motorSpeed2);
  motor3.writeMicroseconds(motorSpeed3);
  motor4.writeMicroseconds(motorSpeed4);

  // Save the current error and time for the next loop
  lastErr = error;
  lastTime = now;

  // Short delay for stability
  delay(10);
}