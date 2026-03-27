# Ball Balancing Table — ESP32 PID Control

**Senior Design Project | 2025**

---

## Overview

A real-time ball balancing system that detects the ball's X-Y position
using a resistive touchscreen and adjusts platform tilt via two servo
motors controlled by a PID algorithm running on an ESP32.
The system includes a live WiFi web dashboard for monitoring and tuning.

---

## System Architecture
```
Resistive Touchscreen → ESP32 (PID) → Servo Motors → Tilting Platform
                           ↕
                      WiFi Dashboard
                    (Live position + FFT
                     + PID tuning GUI)
```

---

## Hardware Components

| Component | Role |
|---|---|
| ESP32 (Freenove WROOM) | Main controller — runs PID, reads touchscreen, drives servos |
| Resistive Touchscreen (26×20 cm) | Detects ball X-Y position via ADC |
| MG996R Servo Motors (x2) | Tilt platform on X and Y axes |
| 5V 10A Power Supply | Powers ESP32 and both servos |

---

## Key Pins

| Signal | ESP32 Pin |
|---|---|
| Touchscreen XP | GPIO32 |
| Touchscreen YP | GPIO33 |
| Servo X | GPIO18 |
| Servo Y | GPIO5 |

---

## PID Parameters (Optimal)

| Parameter | Value |
|---|---|
| Kp | 2.5 – 3.0 |
| Ki | 0.005 – 0.01 |
| Kd | 1.0 – 1.2 |

---

## Performance Results

| Metric | Requirement | Achieved |
|---|---|---|
| Tilt range | ±15° | ±17° ✅ |
| Servo speed | ≥ 90°/s | ~112°/s ✅ |
| Position accuracy | ±0.5 cm | ±0.47 cm ✅ |
| Settling time | ≤ 10 s | ~2.5 s ✅ |

---

## Features

- ✅ Ball balancing at center
- ✅ Predefined path tracking (Circle, Square)
- ✅ Live WiFi web dashboard
- ✅ Real-time PID tuning via browser
- ✅ FFT frequency analysis

---

## How to Run

1. Open `firmware/BallBalancingTable.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module**
3. Upload to ESP32
4. Connect to ESP32 WiFi hotspot
5. Open browser → `192.168.4.1`

---

## Tech Stack

![ESP32](https://img.shields.io/badge/ESP32-Arduino_C++-blue)
![PID](https://img.shields.io/badge/Control-PID-green)
![WiFi](https://img.shields.io/badge/Interface-WiFi_Dashboard-orange)