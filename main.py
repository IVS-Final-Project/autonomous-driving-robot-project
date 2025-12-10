import threading
import time

import globalVar
from taskManager import getCurZoneTask, getObjInfoTask, mainTask
from motorControl import lonControl, latControl

if __name__ == "__main__":
    stop_event = threading.Event()
    args = {}

    # 스레드 생성
    curZoneTaskThread = threading.Thread(
        target=getCurZoneTask, args=(stop_event, args)
    )

    objInfoTaskThread = threading.Thread(
        target=getObjInfoTask, args=(stop_event, args)
    )

    mainTaskThread = threading.Thread(
        target=mainTask, args=(stop_event, args)
    )

    curZoneTaskThread.start()
    objInfoTaskThread.start()
    mainTaskThread.start()

    globalVar.desiredAngle = 50
    globalVar.desiredSpeed = 80
    try:
        while True:

            speed = globalVar.desiredSpeed
            angle = globalVar.desiredAngle

            lonControl(speed)
            latControl(angle)

            time.sleep(0.2)
            
    finally:
        stop_event.set()
        curZoneTaskThread.join(timeout=1)
        objInfoTaskThread.join(timeout=1)
        mainTaskThread.join(timeout=1)
        print("Stopped.")
