EGR 604 Follow Me Cooler Project
================================

Source code for the Raspberry PI Pico controller of a mobile robot. 

Contents
--------
The main code for this project is under the src folder. 

The following subfolders contain test code for specific components of the robot:
- ble (bluetooth communication testing)
- gps (testing GPS connection and NMEA decoding)
- scanner (testing a servo and ultrasonic sensor as an obstacle avoidance scanner)
- magnetometer (testing the compass)
- main (the completed robot control program)

Dependencies
------------

This project uses [MicroNMEA](https://github.com/stevemarple/MicroNMEA), with some edits for RPi Pico SDK compatibility, to decode UART GPS data. 

Program Structure
-----------------

@startuml
node Core0 {
    [Motor Control] as mc
    [Operation Mode] as op
    [Serial Debugging (printf)] as dbg
    
}

node Core1 {
    [User Interface] as ui
    [Scanner Data Processing] as scn
    [GPS Coordinates] as pos
    [Data Packer] as data
}

interface Servo as srv
interface Trig as trig
interface Echp as echo

interface "Bluetooth (UART)" as ble

interface "GPS (UART)" as gps

interface "PWM" as pwm1
interface "PWM" as pwm2
interface "PWM" as pwm3
interface "PWM" as pwm4

queue Queue as q

ble --> ui: User commands and target coordinates
gps --> pos: Current robot coordinates
scr --> srv: Scanner angle setpoints
scn <-- echo >: Distance time-of-flight measurements
scn --> trig: Distance measurement trigger

ui -> data: Updated commands and coordinates from user
pos -> data: Updated robot position
scn -> data: Updated open path data

data --> q: Update messages for core 0

q --> op: Updated data
op -> mc: Target heading and speed

q --> mc: Manual controls

mc -> pwm1: Motor commands
mc -> pwm2: Motor commands
mc -> pwm3: Motor commands
mc -> pwm4: Motor commands
@enduml