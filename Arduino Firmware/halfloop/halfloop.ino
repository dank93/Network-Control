#include "SPI.h"
#include "encoder.h"
#include "driver.h"
#include "irsensor.h"

Driver drivers[] = {
    Driver(28, 29, 5, 44), // Left A
    Driver(24, 25, 3, 42), // Left B
    Driver(22, 23, 2, 48), // Right A
    Driver(26, 27, 4, 46), //  Right B
};

IR_Sensor irleft = IR_Sensor(A0, 1000, 10);
IR_Sensor irright = IR_Sensor(A1, 1000, 10);

void setup() {
  Serial.begin(115200);//37500
  for(int i = 0; i < 4; i++) {
      drivers[i].init(); // initialize PID
      drivers[i].setCountsPerRad(52803.79);//83046.4); // set encoder counts per radian of output shaft
      drivers[i].setTorqueConstant(4.331); // set motor Nm/A at the output shaft
      drivers[i].setCommandCurrentMax(7.0); // current commands are scaled from 0 to this parameter (amps)
      drivers[i].setCurrentLimit(7.0); // limits absolute value of current command (amps)
      drivers[i].setMaxAngle(2000); // if motor is past this angle & torque is positive, torque defaults to 0 (rad)
      drivers[i].setMinAngle(-2000); // if motor is past this angle & torque is negative, torque defaults to 0 (rad)
  }
  irleft.init();
  irright.init();
}

float lthetaLeftA = 0;
float lthetaLeftB = 0;
float lthetaRightA = 0;
float lthetaRightB = 0;
float lirLeft = 0;
float lirRight = 0;

float thetaLeftA = 0;
float thetaLeftB = 0;
float thetaRightA = 0;
float thetaRightB = 0;
float irLeft = 0;
float irRight = 0;
//float torque = 0;
String torque = "999";

float tthetaLeftA = 0;
float tthetaLeftB = 0;
float tthetaRightA = 0;
float tthetaRightB = 0;
float tirLeft = 0;
float tirRight = 0;

float ddtthetaLeftA = 0;
float ddtthetaLeftB = 0;
float ddtthetaRightA = 0;
float ddtthetaRightB = 0;
float ddtirLeft = 0;
float ddtirRight = 0;

float dt;
long t = millis();
long looptime;
long currenttime;

void loop() {
    lthetaLeftA = thetaLeftA;
    thetaLeftA = drivers[0].getCurrentAngle()*100;
    dt = (micros() - tthetaLeftA)/1000000;
    ddtthetaLeftA = (thetaLeftA - lthetaLeftA)/dt;
    tthetaLeftA = micros();

    lthetaLeftB = thetaLeftB;
    thetaLeftB = drivers[1].getCurrentAngle()*100;
    dt = (micros() - tthetaLeftB)/1000000;
    ddtthetaLeftB = (thetaLeftB - lthetaLeftB)/dt;
    tthetaLeftB = micros();

    lthetaRightA = thetaRightA;
    thetaRightA = drivers[2].getCurrentAngle()*100;
    dt = (micros() - tthetaRightA)/1000000;
    ddtthetaRightA = (thetaRightA - lthetaRightA)/dt;
    tthetaRightA = micros();

    lthetaRightB = thetaRightB;
    thetaRightB = drivers[3].getCurrentAngle()*100;
    dt = (micros() - tthetaRightB)/1000000;
    ddtthetaRightB = (thetaRightB - lthetaRightB)/dt;
    tthetaRightB = micros();

    lirLeft = irLeft;
    irLeft = irleft.getDistance()*100;
    dt = (micros() - tirLeft)/1000000;
    ddtirLeft = (irLeft - lirLeft)/dt;
    tirLeft = micros();

    lirRight = irRight;
    irRight = irright.getDistance()*100;
    dt = (micros() - tirRight)/1000000;
    ddtirRight = (irRight - lirRight)/dt;
    tirRight = micros();

    looptime = (millis()-t)*100;
    t = millis();
    currenttime = millis()*100;

    String msg = "s";
    msg  = msg + "," + String(thetaLeftA) + "," + String(ddtthetaLeftA);
    msg  = msg + "," + String(thetaLeftB) + "," + String(ddtthetaLeftB);
    msg  = msg + "," + String(thetaRightA) + "," + String(ddtthetaRightA);
    msg  = msg + "," + String(thetaRightB) + "," + String(ddtthetaRightB);
    msg  = msg + "," + String(irLeft) + "," + String(ddtirLeft);
    msg  = msg + "," + String(irRight) + "," + String(ddtirRight);
    msg  = msg + "," + torque;
    msg  = msg + "," + String(looptime);
    msg  = msg + "," + String(currenttime);
    Serial.println(msg);

    useCommand(); // read, interpret, and execute torque commands
}

void useCommand() {
  if (Serial.available() > 0) {
              Serial.readStringUntil('s');
              String inString = Serial.readStringUntil('\n');
              int limit0 = inString.indexOf('/');
              int limit1 = inString.indexOf('/', limit0+1);
              int limit2 = inString.indexOf('/', limit1+1);
              int limit3 = inString.indexOf('/', limit2+1);
              int limit4 = inString.indexOf('/', limit3+1);
              float torque0 = (inString.substring(0, limit0)).toFloat();
              float torque1 = (inString.substring(limit0+1, limit1)).toFloat();
              float torque2 = (inString.substring(limit1+1, limit2)).toFloat();
              float torque3 = (inString.substring(limit2+1, limit3)).toFloat();
              int incomingchecksum = (inString.substring(limit3+1, limit4)).toInt();
              int check = 0;
              for (int i=0; i<(inString.substring(0, limit3+1)).length(); i++) { // checksum of al data including /'s
                check += (int)(inString[i]);
              }
              if (check == incomingchecksum) {
                drivers[0].sendCommand(torque0);
                drivers[1].sendCommand(torque1);
                drivers[2].sendCommand(torque2);
                drivers[3].sendCommand(torque3);
                torque = String(torque1*100);
              }
              else {
                torque = "nan";
              }
  }
}
