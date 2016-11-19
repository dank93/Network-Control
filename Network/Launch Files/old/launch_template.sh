echo Launching robot scripts!
osascript -e 'tell application "Terminal" to do script "python multipoller.py 5002 50 5003 5004"'
osascript -e 'tell application "Terminal" to do script "python controller_template.py 5003 50 5002"'
osascript -e 'tell application "Terminal" to do script "pythonw gui_template.py 5004 50"'