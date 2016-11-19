echo Launching robot scripts!
start "Controller" python  "../Controllers/controller.py" controlfun_template 5003 1000 4999 5000 5001 5004
start "Torso IMU" python "../Hardware Links/publishimu.py" torso COM37 4999 roll pitch
start "Leg IMU" python "../Hardware Links/publishimu.py" leg COM40 5000 roll pitch
start "Arduinode" python "../Hardware Links/arduinode.py" COM43 5001 5003
start "GUI" pythonw "../GUIs/gui_template.py" 5004 1000
