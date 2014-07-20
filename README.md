Cayley1_Flight_Controller
=========================

This C++ program is used to fly the Cayley 1 quad rotor aircraft via uplaod to its onboard Arduino (hence the .ino flie extensions). Using inputs from an IMU and Ultrasonic sensor the aircraft calculates its Euler angles and self stabilise by passing these though a PID controller and controlling the rotors. User control is also possible from a XBee radio module.
