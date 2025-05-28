
# 🌿 Greenhouse Air Quality Control System

An embedded system built with **Arduino UNO** and **bare-metal AVR C** for monitoring and controlling air quality in a greenhouse. The system measures temperature and gas concentration, triggers an alarm if thresholds are exceeded, and controls ventilation via a servo motor.

## 🛠 Components Used

- Arduino UNO
- Analog Temperature Sensor (TMP36)
- Analog Gas Sensor (e.g., MQ-2)
- Servo Motor
- 16x2 LCD with I2C interface (PCF8574)
- 2 Push Buttons (for menu navigation)
- Red LED for alarm indication
- Various resistors

## 🔌 Wiring Diagram

![Wiring Diagram](./1f9fcc9f-82f5-4473-89e5-70d27542ffe1.png)

## Features

- **Main menu** displayed on LCD, navigated via two buttons (simulated interrupts via polling with debounce)
- **Sensor reading**: temperature (A1) and gas (A0), values shown on LCD
- **Threshold settings**: configurable limits for temperature and gas, stored in RAM
- **Alarm indicator**: red LED lights up when thresholds are exceeded
- **Ventilation control**: PWM-controlled servo motor (0–45°) reacts to how much limits are exceeded
- **UART interface** for debugging and I2C device scanning

## Menu Structure

- `Main Menu`
  - `> Sensor Readings` – Displays current temperature and gas levels
  - `> Set Thresholds` – Allows user to configure temperature and gas thresholds
  - `> Status` – Shows alarm status and ventilation percentage

**Navigation:**
- **Button 1 (D2)**: scroll/select + increase value
- **Button 2 (D3)**: enter/exit/edit

## Source Code

Written entirely in **bare-metal AVR C** (no Arduino libraries). Includes:

- TWI/I2C initialization and communication (PCF8574 LCD in 4-bit mode)
- PWM signal generation via Timer1
- ADC for reading analog sensors
- Button debounce logic
- Menu and UI logic on 16x2 I2C LCD
- UART serial debugging output

> Full source available in [`main.c`](main.c)

## Default Thresholds

- Temperature: **35°C**
- Gas: **900** (raw ADC value)

## Ventilation Logic

When temperature or gas concentration exceeds the threshold:
- Red LED (D13) turns ON
- Servo opens proportionally in the range **0–45°**
  - Based on how far the values exceed the limits

## Compilation & Upload

1. Use Atmel Studio or plain Makefile with `avr-gcc`
2. Compile and flash via `avrdude`

Example:
```bash
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os main.c -o main.elf
avr-objcopy -O ihex main.elf main.hex
avrdude -c arduino -p m328p -P COMx -b 115200 -U flash:w:main.hex
````




