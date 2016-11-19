#include "irsensor.h"
#include "Arduino.h"

///////////////////////////////////////////////////////////////////////////////
// IR_Sensor Class  Functions
///////////////////////////////////////////////////////////////////////////////
// Constructor
IR_Sensor::IR_Sensor(int inpin, int numsamples, int avgsamps)
{
  inputPin = inpin;
  arraysize = numsamples;
  avgsamples = avgsamps;
  avgarray = new float[avgsamples];
  for (int i = 0; i < avgsamples; i++){
    avgarray[i] = 0.0;
  }
}

// init
void IR_Sensor::init() {
  pinMode(inputPin, INPUT);
}

// Get functions
float IR_Sensor::getDistance() {
  // float voltage = analogRead(inputPin);
  // for (int i = 0; i < ARRAYSIZE-1; i++){readings[i] = readings[i+1];}
  // readings[ARRAYSIZE-1] = voltage;
  // float sum = 0.0;
  // for (int i = 0; i < ARRAYSIZE; i++){sum += readings[i];}
  // float avgVoltage = sum/ARRAYSIZE;
  // float cmBetween = 46.764*pow(2.718, avgVoltage*-0.002472);
  // float cm = cmBetween + EXTRACM;
  // return cm;

  float voltages[arraysize];
  for (int i = 0; i < arraysize; i++){voltages[i] = analogRead(inputPin);}
  float sum = 0.0;
  for (int i = 0; i < arraysize; i++){sum += voltages[i];}
  float avgVoltage = sum/arraysize;
  float cmBetween = 15558.9957*pow(avgVoltage,-1.2691445);
  float cm = cmBetween + EXTRACM;

  for (int i = 0; i < avgsamples-1; i++){avgarray[i] = avgarray[i+1];}
  avgarray[avgsamples-1] = cm;
  sum = 0.0;
  for (int i = 0; i < avgsamples; i++){sum += avgarray[i];}
  float avgcm= sum/avgsamples;
  return avgcm;
}
