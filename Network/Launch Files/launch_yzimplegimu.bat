echo Launching robot scripts!
start "Controller" python "../Controllers/controller.py" controlfun_yzimplegimu 5003 1000 4999 5000 5001
start "Left IMU" python  "../Hardware Links/publishimu.py" left COM37 4999 roll pitch rollrate pitchrate
start "Right IMU" python  "../Hardware Links/publishimu.py" right COM40 5000 roll pitch rollrate pitchrate
start "Optoforce" python "../Hardware Links/optoforce.py"
start "Arduinode" python "../Hardware Links/arduinode.py" COM44 5001 5003
start python "../Utilities/plot.py" 5003 y "left roll" x
start "Logger" python "../Utilities/log.py" 5003 yzvid.csv all
