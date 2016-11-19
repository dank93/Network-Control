echo Launching robot scripts!
start "Controller" python "../Controllers/controller.py" controlfun_xzsimple 5003 1000 4999 5000 5001
start "Left IMU" python  "../Hardware Links/publishimu.py" left COM37 4999 roll pitch rollrate pitchrate
start "Right IMU" python  "../Hardware Links/publishimu.py" right COM40 5000 roll pitch rollrate pitchrate
start "Arduinode" python "../Hardware Links/arduinode.py" COM44 5001 5003
start python "../Utilities/plot.py" 5003 "l left"
#start "Logger" python "../Utilities/log.py" 5003 xzvid.csv all
