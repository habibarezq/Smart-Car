# Arduino Smart Car

An Arduino-based smart car controlled via Bluetooth, featuring manual drive, line tracking, body tracking, obstacle avoidance, and voice commands.

## Features

- Manual control via Bluetooth
- Line tracking using IR sensors
- Body tracking (hand following)
- Obstacle avoidance using an ultrasonic sensor
- Voice command support
- LED indicators for turns
- Adjustable motor speed
- Android app for control

## Controller App

We built a custom Android app to control the smart car via Bluetooth.
- APK file: [SmartCar.apk](./SmartCar.apk)

> Download the APK, install it on your Android phone, and pair with the car via Bluetooth.

## Hardware Requirements

- Arduino Uno
- L298N Motor Driver
- 2x DC Motors
- 2x IR Sensors
- Ultrasonic Sensor
- LEDs
- HC-05 or HC-06 Bluetooth Module
- Power supply

## Pin Configuration

| Component           | Arduino Pin        |
|---------------------|---------------------|
| Motor ENA           | D3                  |
| Motor ENB           | D6                  |
| Motor IN1 / IN2     | D2 / D4              |
| Motor IN3 / IN4     | D5 / D7              |
| IR Sensor (Right)   | D8                  |
| IR Sensor (Left)    | D9                  |
| LED (Left)          | D12                 |
| LED (Right)         | D13                 |
| Bluetooth RX / TX   | D10 (RX) / D11 (TX)  |

## Bluetooth Commands

| Command | Action               |
|---------|----------------------|
| `f`     | Move forward          |
| `b`     | Move backward         |
| `l`     | Turn left             |
| `r`     | Turn right            |
| `s`     | Stop                  |
| `t`     | Start line tracking   |
| `m`     | Stop line tracking    |
| `h`     | Start body tracking   |
| `c`     | Stop body tracking    |
| `i`     | Increase speed        |
| `d`     | Decrease speed        |

## Voice Commands

| Command | Action    |
|---------|-----------|
| `^`     | Forward    |
| `-`     | Backward   |
| `<`     | Left       |
| `>`     | Right      |
| `*`     | Stop       |
