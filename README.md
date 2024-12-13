# ADXL345-s-accelerometer-with-motor

Motor Control Using ADXL345 Accelerometer and Encoder

This project demonstrates how to control a motor's position so that it matches the tilt angle read from the ADXL345 accelerometer. The motor position is tracked using an encoder, and the system ensures that the motor rotates to align with the desired tilt angle.

Features

Tilt Angle Calculation: Reads the X-axis tilt angle from the ADXL345 accelerometer and calculates the corresponding angle in degrees.

Motor Position Control: Uses the encoder to track the motor's position in degrees and adjusts the motor to match the desired angle.

Proportional Control: Implements a proportional control system to minimize the error between the desired tilt angle and the motor's position.

Real-Time Monitoring: Displays the tilt angle, motor position, and error values on an LCD screen and the serial monitor.

Components

Arduino board (e.g., Uno, Mega)

ADXL345 accelerometer (I2C interface)

Motor (DC or geared) with an encoder

Motor driver (L293D or similar)

LiquidCrystal-compatible LCD

Connecting wires and breadboard

Wiring Diagram

ADXL345:

SDA -> Arduino A4 (for UNO)

SCL -> Arduino A5 (for UNO)

VCC -> 3.3V or 5V

GND -> GND

Motor:

IN1 -> Arduino pin 9

IN2 -> Arduino pin 10

ENA -> Arduino pin 6

Encoder:

Signal A -> Arduino pin 2

Signal B -> Arduino pin 3

LCD:

RS -> Arduino pin 7

EN -> Arduino pin 8

D4, D5, D6, D7 -> Arduino pins 4, 5, 12, 11

Code Overview

Initialization:

Sets up I2C communication with the ADXL345.

Configures motor control and encoder pins.

Initializes the LCD.

Main Loop:

Reads the tilt angle from the ADXL345.

Calculates the motor's current angular position using encoder data.

Computes the error between the desired and actual angles.

Adjusts the motor's speed and direction to minimize the error.

Displays the tilt angle and motor position on the LCD and serial monitor.

Interrupt Service Routine:

Updates the encoder position to track the motor's rotation in real-time.

Dependencies

This project uses the following Arduino libraries:

Wire.h (for I2C communication with the ADXL345)

LiquidCrystal.h (for LCD display)

Usage

Upload the Code:

Open the code in the Arduino IDE.

Select the correct board and port.

Upload the code to the Arduino.

Run the System:

Connect all components as per the wiring diagram.

Power on the Arduino.

Tilt the ADXL345 and observe the motor aligning its position to match the tilt angle. Check the LCD for real-time updates.

Monitor via Serial:

Open the Serial Monitor in the Arduino IDE.

Set the baud rate to 115200.

Observe the tilt angle and motor position in real-time.

Notes

Ensure the ADXL345 is mounted securely, and the X-axis is aligned with the desired tilt axis.

Adjust the proportional control gain in the code (motorSpeed = constrain(abs(error) * 5, 0, 255);) for smoother operation if necessary.

Verify the encoder's pulses per revolution and update the calculation accordingly.

License

This project is licensed under the MIT License. Feel free to use and modify it as needed.

