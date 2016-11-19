echo Launching robot scripts!
start python publishimu.py torso COM40 5000 100 roll pitch
start python multipoller.py 5002 100 5000 5001 5003
start python arduinode.py COM43 5001 100 5002
start python controller_1ddummyimpedance.py 5003 100 5002