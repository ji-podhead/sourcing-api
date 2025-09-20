# run_uvicorn.py
from twisted.internet import main
main.installSignalHandlers = lambda: None

import uvicorn

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, app_dir="/home/sourcingapi/ros2_ws/src/api/src", reload=True)
    uvicorn.Server.install_signal_handlers(lambda: None ) 
# poetry run python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --app-dir /home/sourcingapi/ros2_ws/src/api/src --reload --reload-dir /home/sourcingapi/ros2_ws/src/api/src
