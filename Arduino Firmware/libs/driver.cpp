#include "Arduino.h"

#ifndef SPI.h
#include "SPI.h"
#endif

#ifndef encoder.h
#include "encoder.h"
#endif

#include "driver.h"

///////////////////////////////////////////////////////////////////////////////
// Driver Class  Functions
///////////////////////////////////////////////////////////////////////////////
// Constructor
Driver::Driver(int enablePin, int directionPin, int setpointPin, int encoderPin)
    : enc(encoderPin) {
    ENABLEPIN = enablePin;
    DIRECTIONPIN = directionPin;
    SETPOINTPIN = setpointPin;
    ENCODERPIN = encoderPin;

    currentAngle = 0;
    commandCurrentMax = 5.0;
    maxAngle = 1024;
    minAngle = 0;
}

void Driver::init() {
    pinMode(ENABLEPIN, OUTPUT);
    pinMode(DIRECTIONPIN, OUTPUT);
    pinMode(SETPOINTPIN, OUTPUT);
    pinMode(ENCODERPIN, OUTPUT);

    digitalWrite(ENABLEPIN, LOW);
    analogWrite(SETPOINTPIN, 0);

    enc.init();
    SPI.begin();
    // enc.clearCount();
}

// Set Functions
void Driver::setCountsPerRad(float cpr) {countsPerRad = cpr;}
void Driver::setTorqueConstant(float tau) {torqueConstant = tau;}
void Driver::setCommandCurrentMax(float ccm) {commandCurrentMax = ccm;}
void Driver::setCurrentLimit(float cl) {currentLimit = cl;}
void Driver::setMaxAngle(float maxA) {maxAngle = maxA;}
void Driver::setMinAngle(float minA) {minAngle = minA;}

// Get Function
float Driver::getCurrentAngle() {
    currentAngle = enc.read()/countsPerRad;
    return currentAngle;
}

// Send Current Command
void Driver::sendCommand(float commandTorque) {
    if (currentAngle > maxAngle && commandTorque > 0) {commandTorque = 0;} // make sure command doesn't exceed limits
    if (currentAngle < minAngle && commandTorque < 0) {commandTorque = 0;}
    float commandCurrent = commandTorque/torqueConstant; // convert to current

    if (commandCurrent > currentLimit) {commandCurrent = currentLimit;} // limit current
    if (commandCurrent < -currentLimit) {commandCurrent = -currentLimit;}
    float output = map(100*commandCurrent, -100*commandCurrentMax, 100*commandCurrentMax, -255, 255); // map to analog output

    if (output >= 0) {digitalWrite(DIRECTIONPIN, LOW);} // set direction pin
    else {digitalWrite(DIRECTIONPIN, HIGH);}

    digitalWrite(ENABLEPIN, HIGH); // enable motor
    analogWrite(SETPOINTPIN, constrain(abs(output), 0, 255)); // set analog setpoint
}
