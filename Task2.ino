#include <Wire.h>
#include <LiquidCrystal.h>

// Define ADXL345 I2C address
#define ADXL345 0x53

// Define motor control pins (connected to L293D)
#define MOTOR_IN1 9
#define MOTOR_IN2 10
#define MOTOR_ENA 6

// Define encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

// Encoder variables
volatile int encoderPosition = 0;
int lastEncoded = 0;

// ADXL345 variables
float tiltAngle = 0.0; // Desired angle from ADXL345
float motorAngle = 0.0; // Motor position from encoder

// Define LCD pins
LiquidCrystal lcd(7, 8, 4, 5, 12, 11); // RS, EN, D4, D5, D6, D7

void setup() {
  Serial.begin(9600);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();

  // Initialize motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);

  // Initialize ADXL345
  Wire.begin();
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D); // Power Control Register
  Wire.write(8);    // Enable measurement
  Wire.endTransmission();

  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Read the tilt angle from ADXL345
  tiltAngle = 2* getTiltAngle();

  // Calculate the current motor position in degrees
  motorAngle = encoderPosition * 360.0 / 750.0; // Assuming 500 encoder pulses per revolution

  // Move motor based on the current angle and target angle
  if (motorAngle < tiltAngle) {
    // Move motor forward (increase angle)
    moveMotorForward();
  } else if (motorAngle > tiltAngle) {
    // Move motor backward (decrease angle)
    moveMotorBackward();
  } else {
    // Stop motor if target angle is reached
    stopMotor();
  }

  // Calculate the error between desired and actual angles
  //float error = tiltAngle - motorAngle;

  // Control the motor to minimize the error
  //controlMotor(error);

  // Display data on LCD
  lcd.setCursor(0, 0);
  lcd.print("Tilt: ");
  lcd.print(tiltAngle);
  lcd.print("   ");

  lcd.setCursor(0, 1);
  lcd.print("Motor: ");
  lcd.print(motorAngle);
  lcd.print("   ");

  Serial.print("Tilt Angle: ");
  Serial.print(tiltAngle);
  Serial.print(" | Motor Angle: ");
  Serial.println(motorAngle);

  //delay(100);
}

void moveMotorForward() {
  // Set motor direction to forward
  analogWrite(MOTOR_ENA, 90);   // Full speed (you can adjust this value)
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

void moveMotorBackward() {
  // Set motor direction to backward
  analogWrite(MOTOR_ENA, 90);  // Full speed (you can adjust this value)
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
}

void stopMotor() {
  // Stop motor movement
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);  // Stop PWM signal to motor
}


// Function to read tilt angle from ADXL345
float getTiltAngle() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start reading from DATAX0 register
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 2, true); // Read 2 bytes for X-axis

  int rawX = 0;
  if (Wire.available() == 2) {
    rawX = (Wire.read() | Wire.read() << 8); // Combine high and low bytes
    if (rawX > 32767) rawX -= 65536; // Convert to signed value
  }

  // Calculate tilt angle in degrees
  float acceleration = rawX / 256.0; // Scale to g (±2g range)
  return atan(acceleration) * 180.0 / 3.141592; // Convert to degrees
}

// Function to control motor based on error
void controlMotor(float error) {
  int motorSpeed = constrain(abs(error) * 5, 0, 255); // Proportional control

  if (error > 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, motorSpeed); // Forward
  } else if (error < 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, motorSpeed); // Reverse
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, 0); // Stop
  }
}

// Encoder interrupt function
void updateEncoder() {
  int MSB = digitalRead(ENCODER_A);
  int LSB = digitalRead(ENCODER_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

  lastEncoded = encoded;
}
