# BME280 ESP-IDF Example

This project demonstrates how to interface the **Bosch BME280** environmental sensor with an ESP32 using **ESP-IDF** and **FreeRTOS**.  
The code reads raw sensor data over I²C, applies Bosch’s official compensation algorithms, and prints **temperature, humidity, and pressure** to the console once per second.

---

## Features

- I²C communication at 400 kHz
- Automatic chip reset & ID verification
- Reads Bosch calibration constants (`dig_T*, dig_P*, dig_H*`)
- Compensation functions for:
  - Temperature (°C)
  - Pressure (Pa)
  - Relative Humidity (%RH)
- Configurable sensor settings:
  - Temperature oversampling: 2×
  - Pressure oversampling: 4×
  - Humidity oversampling: 1×
  - IIR filter: coeff 4
  - Standby time: 125 ms (NORMAL mode only)
- Sensor runs in **NORMAL mode** (periodic conversions)
- Logs values to the console using `ESP_LOGI`
- FreeRTOS task loop (1 s interval)

---

## Hardware

- **ESP32** development board
- **BME280** module (address `0x76` by default, change to `0x77` if needed)
- Pull-up resistors:
  - Internal pull-ups enabled in code
  - It is strongly advised to use external 2.2-10kΩ pull-ups

| Signal | GPIO (ESP32) |
|--------|--------------|
| SDA    | 23           |
| SCL    | 22           |

---
## Console
![Console Example](/docs/Console.png)

---

## Wiring
- 3V3 --> VCC
- GND --> GND
- GPIO23 --> SDA
- GPIO22 --> SCL
