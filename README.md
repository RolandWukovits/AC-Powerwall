# AC-Powerwall *current software version 1.00.39/14.10.2020* Incl Wifi and Blynk

This repository contains code and documentation files from my YouTube "The AC-Powerwall Project" on "Roland W" channel.
Playlist: https://www.youtube.com/playlist?list=PLz3GU7wMX4_P-K7Ht42iucupu2CXGZh10
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
- 20A AC-current sensor module to be connected on the inverter AC connection
- ESP01 Module (ESP8266)
- AMS1117 3,3V Version Buck converter to power ESP01
- 12 way Rotary Switch as Mode selector
- 13x 10k, 2x 1k, 1x 2k, 1x 830R 1/4W resistors
- diverse male, female headers and sockets

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
- Battery logic to be customized. You will have to enter your particular voltage curve of your battery. SOC can be 
  calculated with enough precision even for LIFEPO chemistry but requires some effort :) You would still have to use a
  Coulomb-meter for monitoring, but you can reset it from information you get from the controller.

