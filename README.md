EGR 604 Follow Me Cooler Project
================================

Source code for the Raspberry PI Pico controller of a mobile robot. 

Contents
--------
The following folders contain test code for specific components of the robot:
- ble (bluetooth communication testing)
- gps (testing GPS connection and NMEA decoding)
- scanner (testing a servo and ultrasonic sensor as an obstacle avoidance scanner)

Dependencies
------------

This project uses [MicroNMEA](https://github.com/stevemarple/MicroNMEA), with some edits for RPi Pico SDK compatibility, to decode UART GPS data. 