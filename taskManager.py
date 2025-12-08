import globalVar
from time import sleep

zoneID = {'IDLE' : 0, 'CHILD' : 1, 'HIGHACCIDENT' : 2, 'SPEEDBUMP' : 3}
vel_ChildZone= 30 # 어린이 보호구역 목표 속도
vel_HighAccidentZone = 50 # 사고다발지역 목표 속도
vel_SpeedBump = 20 # 과속 방지턱 목표 속도
vel_ObjInRoad = 10 # 도로 밖에서 장애물을 인식했을 때 목표 속도

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
        if detected:
            if inRoad:
                speed = 0
            else:
                speed = vel_ObjInRoad
        
        elif zone == zoneID['IDLE']:
            continue
        elif zone == zoneID['CHILD']:
            speed = vel_ChildZone
        elif zone == zoneID['HIGHACCIDENT']:
            speed = vel_HighAccidentZone
        elif zone == zoneID['CHILD']:
            speed = vel_SpeedBump
        
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
