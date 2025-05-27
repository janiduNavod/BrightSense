# 🌞 BrightSense: Smart Ambient Lighting and Temperature Control System

BrightSense is an energy-efficient home automation system developed to dynamically adjust indoor lighting based on ambient sunlight and manage temperature-responsive ventilation. This project was proudly presented at an IEEE Society competition, showcasing a practical and sustainable solution to reduce energy waste.

## 🔍 Overview

BrightSense uses sensors and smart control algorithms to:
- 🌤 Detect ambient light levels and adjust indoor lighting brightness using a PID controller.
- 🌡 Measure indoor temperature and control a servo-driven ventilation mechanism (like window blinds).
- 🚶 Detect motion to ensure lights are only active when someone is present.
- 🎛 Allow manual adjustment of light sensitivity via a potentiometer.
- 📡 Enable IR remote-based control.
- 📺 Display live sensor readings and system status on an I2C LCD.

## ⚙️ Features

- **Ambient Light Detection** (APDS-9930 sensor)
- **Temperature & Humidity Monitoring** (DHT11 sensor)
- **Motion Detection** (PIR Sensor)
- **Dynamic Lighting Control** using PWM and PID
- **Temperature-Controlled Ventilation** via Servo Motor
- **LCD User Interface** with icons for light, temperature, humidity
- **IR Remote Input Support**
- **Energy-saving logic**: lights only on when needed

## 🧰 Components Used

| Component                | Description                        |
|--------------------------|------------------------------------|
| Arduino UNO              | Microcontroller                    |
| APDS-9930                | Ambient light and proximity sensor |
| DHT11                    | Temperature & humidity sensor      |
| PIR Motion Sensor        | Detects human presence             |
| IR Receiver              | Accepts commands from remote       |
| Servo Motor              | Controls window/ventilation        |
| MOSFET + Bulb            | PWM brightness control             |
| I2C LCD (16x2)           | Displays system status             |
| Potentiometer            | Adjusts light threshold sensitivity|

## 🧠 PID Controller

The PID controller is used to regulate bulb brightness



![Doc2](https://github.com/user-attachments/assets/9e41f98d-4d9e-4277-bddc-deb71a15fd22)
