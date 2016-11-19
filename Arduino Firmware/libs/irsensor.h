#include "Arduino.h"

#ifndef irsensor
#define irsensor

#define EXTRACM 0

class IR_Sensor{
  public:
    IR_Sensor(int inpin, int numsamples, int avgsamps); // analog input pin
    void init();
    float getDistance(); // m

  private:
    int inputPin;
    int arraysize;
    int avgsamples;
    float *avgarray;
    // float readings[ARRAYSIZE];
};

#endif
