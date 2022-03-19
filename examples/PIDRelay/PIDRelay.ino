#include <Adafruit_MAX31865.h>
#include <StuPID.hpp>

StuPIDRelay *PID = nullptr;

bool RelayState = LOW;

double Kp = 0.08;
double Ki = 0.0001;
double Kd = 2.0;
double WindowSize = 3000.0;

double Setpoint = 100.0;
double Input = 0.0;
uint8_t OutputSSRPIN = 3;
uint8_t TemperatureSensorPin = A0;

// Thermocouple *********************************************************************
#define MAX31865_TYPE MAX31865_2WIRE // set to 3WIRE or 4WIRE as necessary
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 1000.0
Adafruit_MAX31865 *Thermocouple = nullptr;

void setup()
{
    Serial.begin(57600);
    while (!Serial)
        ;
    Serial.println("Start!");
    
    pinMode(OutputSSRPIN, OUTPUT);
    // Relay pin starts low
    digitalWrite(OutputSSRPIN, RelayState);

    Thermocouple = new Adafruit_MAX31865(TemperatureSensorPin);
    Thermocouple->begin(MAX31865_TYPE);

    PID = new StuPIDRelay(&Input, &Setpoint, &RelayState, WindowSize, &Kp, &Ki, &Kd);
}

void loop()
{
    Input = Thermocouple->temperature(RNOMINAL, RREF);
    PID->run();
    Serial.println(String(">Input:") + Input);
    Serial.println(String(">Relay:") + RelayState);
    digitalWrite(OutputSSRPIN, RelayState);
}
