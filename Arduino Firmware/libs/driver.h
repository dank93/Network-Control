#include "Arduino.h"

#ifndef SPI.h
#include "SPI.h"
#endif

#ifndef encoder.h
#include "encoder.h"
#endif

#ifndef driver
#define driver

class Driver {
  public:
    Driver(int enablePin, int directionPin, int setpointPin, int encoderPin);
    void init();
    void setCountsPerRad(float cpr);
    void setTorqueConstant(float tau); // Nm/A
    void setCommandCurrentMax(float ccm); // MUST AGREE WITH MAXON DRIVER CURRENT MAX
    void setCurrentLimit(float cl); // A
    void setMaxAngle(float maxA); // rad
    void setMinAngle(float minA); // rad
    float getCurrentAngle(); // rad
    void sendCommand(float commandTorque);

  private:
    int ENABLEPIN;
    int DIRECTIONPIN;
    int SETPOINTPIN;
    int ENCODERPIN;
    encoder enc;

    float countsPerRad;
    float torqueConstant;
    float currentAngle;
    float currentLimit;
    float commandCurrentMax;
    float maxAngle;
    float minAngle;
};

#endif
