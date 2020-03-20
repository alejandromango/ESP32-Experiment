# ESP32 Experiment
 A testbed software for the Maslow 5 motor driver board with esp32 control

## Interface
The device hosts a web interface with to provide current motor position data, and change some control settings

### Motor Controls
Each motor can be controlled by setting its setpoint. The units of the setpoint change based on the control mode

### PID Tuning
The PID tuning inputs set the PID parameters for all motors in the current mode. These persist when changing modes, but not on reboot

### Control Modes
An integer input is required to change control modes. There are four options:
    * 0 - REVOLUTIONS: Sets the position of the motor in number of revolutions from 0
    * 1 - CURRENT: Sets a constant current in mA for the motor to maintain
    * 2 - DISTANCE: Sets the position of the motor in distance in mm from 0
    * 3 - SPEED: Sets to speed of the motor in mm/second