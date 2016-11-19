// Script that runs 4 PID controllers
// Daniel Kurek, June 2016
// d'Arbeloff Lab, MIT

#include "SPI.h"
#include "encoder.h"
#include "maxon_pid.h"

PID controllers[] = {
    PID(28, 29, 5, 44), // Left A
    PID(24, 25, 3, 42), // Left B
    PID(22, 23, 2, 48), // Right A
    PID(26, 27, 4, 46), //  Right B
};

void setup() {
    Serial1.begin(115200); // open serial connection
    Serial1.flush();
    for(int i = 0; i < 4; i++) {
        controllers[i].init(); // initialize PID
        controllers[i].setCountsPerRad(83046.4); // set encoder counts per radian of output shaft
        controllers[i].setTorqueConstant(4.331); // set motor Nm/A at the output shaft
        controllers[i].setCommandCurrentMax(5.0); // current commands are scaled from 0 to this parameter (amps)
        controllers[i].setMaxAngle(0.20); // if motor is past this angle & torque is positive, torque defaults to 0 (rad)
        controllers[i].setMinAngle(-0.20); // if motor is past this angle & torque is negative, torque defaults to 0 (rad)
        controllers[i].setKP(0.0); // Nm/rad
        controllers[i].setKI(0.0); // Nm/rad/sec
        controllers[i].setKD(0.0); // Nm-sec/rad
    }
}

float t0 = millis();
float positionRef = 0; // Set goal position (rad)
float elapsedMillis = 50; // Determine time step between loops

void loop() {
     if (Serial1.available() >= 3) { // check is settings message was received
         parseSettings(); // parse message and implement settings
     }
    if (millis() - t0 > elapsedMillis) { // if time step has passed
        t0 = millis();
        for (int i = 0; i < 4; i++) {
            controllers[i].run(elapsedMillis); // Run the controllers
        }
//         sendData();
    }
}

void sendData() {
    Serial1.print("Hello World");
}

void parseSettings() {
    char char1 = Serial1.read(); // not sure if comes as ascii int or actual char
    if (char1 == 'p') { // if p gain is being set
        int controllerNum = Serial1.read(); // read which controller is being set
        int pGain = Serial1.read(); // read p gain float value
        controllers[controllerNum].setKP(pGain); // set value
        Serial1.println(controllerNum);
        Serial1.println(pGain);
    }
    else if (char1 == 'd') { // if d gain is being set
        int controllerNum = Serial1.read(); // read which controller is being set
        int dGain = Serial1.read(); // read d gain float value
        controllers[controllerNum].setKD(dGain); // set value
    }
    else if (char1 == 'r') { // if position reference is being sent
        Serial1.flush();
        for (int i=0; i<4; i++) {
            controllers[i].setPosition(controllers[i].getCurrentAngle());
        }
    }
}
