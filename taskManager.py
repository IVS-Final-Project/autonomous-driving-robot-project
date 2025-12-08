import globalVar
from time import sleep

def getCurZoneTask(stop_event, 인자들):
    while not stop_event.is_set():
        globalVar.zoneInfo = getCurZone(인자)
        sleep(0.1)

def getObjInfoTask(stop_event, 인자들):
    while not stop_event.is_set():
        globalVar.isObjDetected, globalVar.isObjInRoad = getObjInfo(인자)
        sleep(0.1)

# 이게 메인 로직
def mainTask(stop_event, 인자들):
    
    while not stop_event.is_set():
        zone = globalVar.zoneInfo
        detected = globalVar.isObjDetected
        inRoad = globalVar.isObjInRoad
        # 여기에 짜면 돼
        
        globalVar.desiredSpeed = speed
        sleep(0.1)


def getCurZone(인자):
    # 여기 짜면 돼
    zoneInfo = 0
    return zoneInfo

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
