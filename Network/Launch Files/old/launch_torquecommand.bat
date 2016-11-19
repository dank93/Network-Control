echo Launching robot scripts!
start python multipoller.py 5002 50 5001 5003 5004
start python controller_torquecommand.py 5003 50 5002
start python gui_torquecommand.py 5004 50
start python arduinode.py COM43 5001 50 5002