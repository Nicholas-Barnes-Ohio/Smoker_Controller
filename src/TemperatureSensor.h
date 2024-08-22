// #ifndef TEMPERATURESENSOR_H
// #define TEMPERATURESENSOR_H
// #endif // TEMPERATURESENSOR_H

// #ifndef WEBSERVER_H
// #define WEBSERVER_H
// // Declare the setupWebserver function
// void loopWebserver(); // Declaration of loopWebserver
// #endif // WEBSERVER_H

// #include <Adafruit_MAX31865.h>
// #include "PID_v1.h" // Include the PID library header



// // Use software SPI: CS, DI, DO, CLK
// extern Adafruit_MAX31865 thermo;

// // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
// #define RREF      430.0
// // The 'nominal' 0-degrees-C resistance of the sensor
// // 100.0 for PT100, 1000.0 for PT1000
// #define RNOMINAL  100.0

// // Define the PID tuning parameters
// extern double Kp, Ki, Kd;

// // Define the PID variables
// extern double Setpoint, Input, Output;

// // Specify the output limits
// extern double MinOutput, MaxOutput;

// // Global variable to track the state of pin 8 (heater)
// extern bool pin8State;

// // Initialize the PID controller
// extern PID myPID;

// void setup();
// // Add any other function declarations you want to expose



