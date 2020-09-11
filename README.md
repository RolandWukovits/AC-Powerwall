# AC-Powerwall *current software version 1.00.021/10.09.2020* Wifi not yet included!

This repository contains code and documentation files from my YouTube "The AC-Powerwall Project" on "Roland W" channel. 
It is meant for a controller based on a Mega 2560, and files can be used freely for private purpose!
While it is appreciated to give an appropriate donation if you have used this code for your work. Thx

The project is still ongoing!!! Files are constantly changing while code is still developed. Please ask for details on progress!

Items/Modules/Devices the controller is using are:
- Arduino Mega or compatible 2560 board
- DS3231 Real Time Clock
- Max485 Modbus Module
- DHT11 Temperature and Humidity sensor
- 2004 LED Screen with Serial Board
- ADS1115 16-bit Analog to Digital Converter
- 6-way Relay Module (5V, activate on LOW)
- Rotary Encoder Switch
- 3x Mosfet Module to switch 12V LED-Indicators
- 3 12V LED-Indicators (red, yellow, green)
- SD-Card Module
- ESP01 Module (ESP8266)
- 12 way Rotary Switch as Mode selector
- 13x 10k, 2x 1k, 1x 2k, 1x 830R 1/8W resistors
- divers male, female headers and sockets

The controller is switching:
- 2 battery chargers (AC small vehicle chargers, use 2 different power outputs)
- 1 inverter via DC-contactor, including a precharge bypass
- Fans

Logic:
- 2 metered modes, by reading out a meter via Modbus
- 1 Timer mode using 2 programmable timers
- 1 Mixed metered/timer mode (M1/T1)
- 4 manual modes (3 for chargers, 1 for discharge)
- 4 setting modes (limits, parameters, timers, clock)

