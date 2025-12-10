📘 BPLFC – Budget Pilot Light Flight Controller
A Low-Cost DIY Fixed-Wing Flight Controller Based on STM32F103C8T6

BPLFC (Budget Pilot Light Flight Controller) is an open-source fixed-wing flight controller built around the affordable STM32F103C8T6 (Blue Pill) board.
It is designed for DIY hobbyists, RC/FPV pilots, makers, and developers who enjoy building and coding their own hardware.

The project aims to be:
✔ Super low-cost using easy-to-find parts
✔ Easy to code with clean and modular firmware
✔ Modular and expandable with common cheap sensors
✔ Fixed-wing focused—no quadcopter complexity
✔ FPV friendly with CRSF/ELRS support

✨ Key Features
🔧 Hardware (Low Budget)

MCU: STM32F103C8T6 (Blue Pill)

IMU: MPU-6050 / MPU-6500 / ICM-20602 (optional, inexpensive)

Barometer: BMP280 / GY-BMP280 (optional)

GPS Module: NEO-6M / NEO-7M (optional)

Receiver: ExpressLRS (CRSF protocol via UART)

Outputs: PWM servo control (aileron, elevator, rudder, throttle)

Status LEDs (FC + CRSF link)

Powered by standard 5V BEC

🛫 Fixed-Wing Flight Modes

Manual passthrough

Gyro-stabilized mode

Level mode (optional if IMU supports)

Basic failsafe (CRSF loss)

🔌 Receiver Support

CRSF over UART (ExpressLRS)

16-channel CRSF decoding

Channel mapping configurable

⚙️ Software

Lightweight clean code

UART interrupts for CRSF

Simple IMU filter

Fixed-wing mixer

Adjustable control loop rate

📡 System Architecture
[ELRS Receiver] -- CRSF UART --> [STM32F103] -- PWM --> Servos
                             \
                              \-- IMU (I2C)
                              \-- Barometer (I2C)
                              \-- GPS (optional UART)

🧩 Wiring Guide
Function	STM32 Pin	Notes
CRSF / ELRS RX	PA10 (USART1 RX)	420 kbaud
IMU I2C SCL	PB6	I2C
IMU I2C SDA	PB7	I2C
Servo PWM 1–4	PA0, PA1, PA2, PA3	Timer 2
Status LED	PC13	Active LOW
GPS (optional)	USART2	Optional
🚀 Quick Start
1. Flash the Firmware

Using ST-Link V2 or USB-Serial (DFU Bootloader):

make flash


Or flash the .bin using STM32CubeProgrammer.

2. Configure Your ELRS Receiver

Set outputs to CRSF:

CH1: Aileron

CH2: Elevator

CH3: Throttle

CH4: Rudder

CH5: Flight Mode

CH6: Stabilize toggle

3. Install on Aircraft

Mount IMU near the CG

Ensure proper IMU orientation

Use stable 5V BEC

📁 Project Structure
/src
 ├── crsf.c/.h       // CRSF parser (ELRS)
 ├── imu.c/.h        // IMU driver
 ├── mixer.c/.h      // Fixed-wing mixer
 ├── pwm.c/.h        // PWM servo output
 ├── fc.c            // Flight control loop
 └── utils/          // Helpers, filters, math

✈️ Fixed-Wing Logic
✔ Gyro Stabilization

Smooth attitude damping (roll & pitch).

✔ Level Mode

Uses accelerometer if available.

✔ Servo Mixing

Default mixer:

aileron  = roll * gain
elevator = pitch * gain
rudder   = yaw * gain
throttle = passthrough / curve

✔ Stable Servo Output

Stable 1000–2000 µs pulses via Timer2.

🛠️ Recommended Parts
Minimum (No Auto-Level)

STM32F103C8T6

ExpressLRS receiver

MPU-6050

5V BEC

4× servo

Full Setup

MPU-6050 / ICM-20602

BMP280 barometer

NEO-6M GPS

Status LED

LC power filter

🧪 Project Status

 CRSF parsing stable

 Servo PWM stable

 Fixed-wing mixer

 Gyro stabilization

 Level mode

 Advanced failsafe

 OSD support (future)

 Basic autopilot (RTH/FBWA-lite)

👨‍💻 Contributing

Contributions are welcome:

Bug fixes

CRSF improvements

New sensors

Mixer improvements

New flight modes

Repository:
https://github.com/pehfpv/bplfc

📜 License

MIT License — free for hobby, education, and experimentation.

❤️ Who Is This Project For?

RC fixed-wing enthusiasts

FPV pilots who love hacking / coding

Electronics & engineering students

Open-source firmware developers

Anyone who wants a simple, hackable flight controller
