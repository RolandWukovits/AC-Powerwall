# AC-Powerwall *current software version 1.02.08/30.12.2020*

This repository contains code and documentation files from my YouTube "The AC-Powerwall Project" on "Roland W" channel.
Playlist: https://www.youtube.com/playlist?list=PLz3GU7wMX4_P-K7Ht42iucupu2CXGZh10
It is meant for a controller based on a Mega 2560, and files can be used freely for private purpose, while it is appreciated to give an appropriate donation if you have used this code for your work. Thx

The project is still ongoing!!! Files are constantly changing while code is still developed. Please ask for details on progress!

Items/Modules/Devices the controller (Board Version 1 and 2) is using are:
- Arduino Mega or compatible 2560 board (Mega 2560 Pro board is used in the controller)
- DS3231 Real Time Clock
- Max485 Modbus Module
- DHT11 Temperature and Humidity sensor
- 2004 LED Screen with I2C Serial Board
- 0,96 inch OLED Display I2C
- ADS1115 16-bit Analog to Digital Converter
- 6-way Relay Module (5V, activate on LOW)
- Rotary Encoder Switch
- 3 12V LED-Indicators (red, yellow, green)
- SD-Card Module
- 20A AC-current sensor module to be connected on the inverter AC connection
- ESP01 Module (ESP8266)
- 12 way Rotary Switch as Mode selector
- diverse 1/4W resistors and electrolytic capacitors
- diverse male, female headers and sockets

The controller is switching:
- 2 or 3 battery chargers (AC small vehicle chargers (max 10A on AC side), use 2 different power outputs (ideally if Charger 1 power output is half of Charger 2 power output), Charger 3 is optional, and is creating a virtual Charger together with Charger 2)
- 1 inverter via DC-contactor, including a precharge bypass
- Fans

Logic:
- 2 metered modes, by reading out a SDM meter via Modbus (SDM120, SDM630, X835, etc)
- 1 Timer mode using 2 programmable timers
- 1 Mixed metered/timer mode (M1/T1)
- 4 manual modes (3 for chargers, 1 for discharge)
- 4 setting modes (limits, parameters, timers, clock)
- Battery logic to be customized. You will have to enter your particular voltage curve of your battery. SOC can be 
  calculated with enough precision even for LIFEPO chemistry but requires some effort :) You would still have to use a
  Coulomb-meter for more exact monitoring, but you can reset it from information you get from the controller.

Monitoring:
- The controller is connecting to the IoT platform Blynk
