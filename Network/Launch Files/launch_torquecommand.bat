echo Launching robot scripts!
start "Controller" python  "../Controllers/controller.py" controlfun_template 5003 1000 5001
start "Arduinode" python "../Hardware Links/arduinode.py" COM44 5001 5003
