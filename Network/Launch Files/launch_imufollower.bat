echo Launching robot scripts!
start "Controller" python "../Controllers/controller.py" controlfun_imufollower 5003 1000 4999 5001
start "IMU" python "../Hardware Links/publishimu.py" arm COM37 4999 roll pitch
start "Arduinode" python "../Hardware Links/arduinode.py" COM44 5001 5003
