echo Launching robot scripts!
start "Controller" python  "../Controllers/controller.py" controlfun_pid 5003 1000 5001 5004
start "Arduinode" /REALTIME python "../Hardware Links/arduinode.py" COM44 5001 5003
start "Logger" python "../Utilities/log.py" 5003 test.csv  "theta left b"
