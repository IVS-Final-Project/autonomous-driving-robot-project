import threading
from time import sleep

# 대충 핀 설정

stop_event = threading.Event()

def getCurZoneTask(stop_flag: threading.Event, 추가로 인자들):
    while not stop_flag.is_set():
        ZoneInfo = getCurZone(인자)
        sleep(0.1)

def getObjInfoTask(stop_flag: threading.Event, 추가로 인자들):
    while not stop_flag.is_set():
        isObjDetected, isObjInRoad = getObjInfo(인자)
        sleep(0.1)

# 이게 메인 로직
def mainTask(stop_flag: threading.Event, 추가로 인자들):
    while not stop_flag.is_set():
        # 여기에 짜면 돼
        sleep(0.1)

def getCurZone(인자):
    # 여기 짜면 돼
    ZoneInfo = 0
    return ZoneInfo

def getObjDetected(인자):
    # 여기 짜면 돼
    isObjDetected = 0
    return isObjDetected

def getObjInRoad(인자):
    # 여기 짜면 돼
    isObjInRoad = 0
    return isObjInRoad

def getObjInfo(인자):
    # 여기 짜면 돼
    
    # 센서퓨전
    isObjDetected = getObjDetected(인자)
    isObjInRoad = getObjInRoad(인자)
    return isObjDetected, isObjInRoad


if __name__ == "__main__":
    
    curZoneTaskThread = threading.Thread(인자들)
    
    ObjInfoTaskThread = threading.Thread(인자들)
    
    mainTaskThread = threading.Thread(인자들)
    
    curZoneTaskThread.start()
    ObjInfoTaskThread.start()
    mainTaskThread.start()
    
    try:
        while True:
            sleep(1.0)
    
    finally:
        stop_event.set()
        curZoneTaskThread.join(timeout=1.0)
        ObjInfoTaskThread.join(timeout=1.0)
        mainTaskThread.join(timeout=1.0)
        print("Stopped.")