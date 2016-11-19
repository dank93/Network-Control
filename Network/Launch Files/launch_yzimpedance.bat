echo Launching robot scripts!
start "Controller" python "../Controllers/controller.py" controlfun_yzimpedance 5003 1000 4999 5001 5004
start "Torso IMU" python  "../Hardware Links/publishimu.py" torso COM37 4999 roll pitch rollrate pitchrate
start "Arduinode" python "../Hardware Links/arduinode.py" COM44 5001 5003
