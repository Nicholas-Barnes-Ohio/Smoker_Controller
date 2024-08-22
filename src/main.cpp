/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 

#include <Arduino.h>
#include <wire.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
//#include <LCD.h>
#include <LCD-I2C.h> //https://registry.platformio.org/libraries/hasenradball/LCD-I2C/examples/Hello_World/Hello_World.ino
#include <LiquidCrystal_I2C.h> // Library for LCD
#include "LCDSettings.h"
#include "Encoder.h"// Include the NewEncoder library

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(9, 10, 11, 12);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// Define the PID tuning parameters
double Kp = 2.0, Ki = 0.2, Kd = 0.2;

// Define the PID variables
double Setpoint = 50.0; // Target temperature
double Input, Output;

// Specify the output limits
double MinOutput = 0, MaxOutput = 255;

// Global variable to track the state of pin 8 (heater)
bool pin8State = LOW; // Assuming the heater is OFF initially

// Initialize the PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Encoder myEnc(2, 3);


void setup() {
  
  //NewEncoder::EncoderState state;


  initializeLCD();
  //Serial.begin(115200);
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  // Initialize the heater control pin
  pinMode(8, OUTPUT);
  digitalWrite(8, pin8State); // Ensure initial state matches pin8State
  //pinMode(9, OUTPUT); // Initialize pin 9 as an output for the debug feature

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(MinOutput, MaxOutput);
 


}

long oldPosition  = -999;

void loop() {

  // Display the setpoint
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);

  delay(50); // Small delay to debounce
  // Read the current temperature
  Input = thermo.temperature(RNOMINAL, RREF);

  // Update the PID controller
  myPID.Compute();

  // Determine the desired state based on PID output
  // Assuming Output > 0 means heater should be on
  bool desiredState = Output > 0;

  static unsigned long lastChangeTime = 0;
  unsigned long currentTime = millis();
  bool shouldChangeState = (currentTime - lastChangeTime) >= 1000; // Check if at least one second has passed

  // Change the state of pin 8 if needed and allowed by the timing constraint
  if (desiredState != pin8State && shouldChangeState) { // Use pin8State to compare desired and current state
    digitalWrite(8, desiredState ? HIGH : LOW); // Set pin 8 to the new state based on PID output
    pin8State = desiredState; // Update pin8State to reflect the new state
    lastChangeTime = currentTime; // Update the last change time
  }

  // Debug feature: Toggle pin 9 state with at least 1 second interval
  static unsigned long lastToggleTime9 = 0;
  static bool pin9State = LOW;
  
  if (currentTime - lastToggleTime9 >= 1000) { // Check if at least one second has passed for pin 9
    pin9State = !pin9State; // Toggle the state
    digitalWrite(9, pin9State ? HIGH : LOW); // Set pin 9 to the new state
    lastToggleTime9 = currentTime; // Update the last toggle time for pin 9
  }

  uint16_t rtd = thermo.readRTD();

  //Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  //Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));
  Serial.print("Setpoint = ");  Serial.println(Setpoint);
  Serial.print(" | PID Output: ");  Serial.println(Output);
  //Serial.print("Current Output on Pin 8: ");
  //Serial.println(pin8State ? "HIGH" : "LOW");
  //Serial.print("setpointEditingMode: ");
  //Serial.println(setpointEditingMode);


  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
 
  Serial.println();
    lcd.setCursor(0, 0); // Setting the cursor in the desired position.
    lcd.print("Temperature = "); 
    lcd.setCursor(14, 0); // Or setting the cursor in the desired position.
    float temperature = thermo.temperature(RNOMINAL, RREF);
    temperature = roundf(temperature * 10) / 10.0; // Correctly rounded to 1 decimal place
    lcd.print(temperature, 1); // Correctly display temperature with 1 decimal place
    lcd.setCursor(0, 1);
    lcd.print("SetPoint = "); 
    lcd.setCursor(11, 1);
    lcd.print(Setpoint);

    lcd.backlight();
    //lcd.clear();

    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      static unsigned long lastComparisonTime = 0;
      Serial.println(newPosition);
      if (currentTime - lastComparisonTime >= 100) {
        lastComparisonTime = currentTime;

        if (newPosition > oldPosition + 4) {
        Setpoint += 2.0; // Add 2 degrees to setpoint
        } else if (newPosition < oldPosition - 4) {
        Setpoint -= 2.0; // Remove 2 degrees from setpoint
        }
        oldPosition = newPosition;
      }}

   

}
