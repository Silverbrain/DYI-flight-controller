# Arduino Flight Controller

This repository contains the code for an Arduino-based flight controller for a quadcopter. The flight controller uses a 6-axis gyro/accelerometer (MPU6050) and controls four brushless motors via PWM signals. Additionally, it incorporates remote controller inputs for throttle, pitch, roll, and yaw.

## Features

- **MPU6050 Integration**: Reads data from the 6-axis gyro/accelerometer to stabilize the quadcopter.
- **PWM Motor Control**: Controls four brushless motors using PWM signals.
- **Remote Controller Input**: Incorporates inputs from an RC receiver for manual control.

## Hardware Required

- Arduino board (e.g., Arduino Uno)
- MPU6050 sensor
- Four brushless motors with ESCs
- RC receiver
- Jumper wires and a breadboard

## Connections

1. **MPU6050**:

   - SDA to Arduino A4
   - SCL to Arduino A5
   - VCC to 3.3V or 5V (depending on the module)
   - GND to GND
2. **ESCs (Electronic Speed Controllers)**:

   - Connect signal wires to PWM output pins:
     - Motor 1 to pin 9
     - Motor 2 to pin 10
     - Motor 3 to pin 11
     - Motor 4 to pin 12
   - Connect power and ground wires to the battery and ground, respectively.
3. **RC Receiver**:

   - Connect the output channels to the input pins:
     - Throttle to pin 2
     - Roll to pin 3
     - Pitch to pin 4
     - Yaw to pin 5

## Code Explanation

### Libraries Used

- **Wire.h**: For I2C communication with the MPU6050 sensor.
- **Servo.h**: For controlling the brushless motors via PWM signals.
- **MPU6050.h**: For interfacing with the MPU6050 sensor.
- **PinChangeInt.h**: For handling pin change interrupts on the RC input pins.

### Code Structure

- **Global Variables**: Define pin numbers, create instances for MPU6050 and Servo, and initialize PID variables.
- **readRCInputs()**: Function to read the pulse width from the RC receiver channels.
- **setup()**: Initialize I2C communication, MPU6050, motor control pins, RC input pins, and PID variables.
- **loop()**: Main loop to read RC inputs, read sensor data, compute PID output, adjust motor speeds, and write speeds to the motors.

### Key Functions

- **readRCInputs()**: Reads the pulse width from the RC receiver channels.
- **setup()**: Sets up the I2C communication, initializes MPU6050, attaches motors, initializes RC input pins, and sets initial motor states.
- **loop()**: Continuously reads RC inputs and sensor data, computes the PID output, and adjusts motor speeds accordingly.

## Usage

1. Power up the quadcopter and the RC transmitter.
2. The flight controller will initialize and stabilize the quadcopter based on the MPU6050 sensor data.
3. Use the RC transmitter to control throttle, pitch, roll, and yaw.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- [MPU6050 Library](https://github.com/jrowberg/i2cdevlib)
- [Servo Library](https://www.arduino.cc/en/Reference/Servo)
