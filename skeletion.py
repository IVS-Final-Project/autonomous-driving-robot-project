import threading
from time import sleep

#getCurZone
import cv2
import numpy as np
from picamera2 import Picamera2
import time
    

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters_create()
    
# 대충 핀 설정

stop_event = threading.Event()

def getCurZoneTask(stop_flag: threading.Event, 추가로 인자들):
    while not stop_flag.is_set():
        zoneInfo = getCurZone(인자)
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
    ZONE_MAP = {
    0: "Normal Zone",
    1: "Children Protection Zone",
    2: "Accident-Prone Zone",
    3: "Bump Zone",
    }
    
    # Default: no zone detected
    zone_id = -1

    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(
        frame, aruco_dict, parameters=aruco_params
    )

    if ids is not None:
        # Draw detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        marker_id = int(ids[0][0])
        corner = corners[0][0]

        cx = int(np.mean(corner[:, 0]))
        cy = int(np.mean(corner[:, 1]))

        # Determine zone ID: if marker_id is in ZONE_MAP → use it, else -1
        if marker_id in ZONE_MAP:
            zone_id = marker_id
            zone_name = ZONE_MAP[marker_id]
        else:
            zone_id = -1
            zone_name = "Unknown Zone"

        # Display zone info on the frame
        cv2.putText(
            frame,
            f"{zone_name} (ID:{zone_id})",
            (cx - 60, cy - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        # Draw corner points
        for pt in corner:
            x, y = int(pt[0]), int(pt[1])
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)

        print(f"Detected Marker ID: {marker_id} → Zone ID: {zone_id} ({zone_name})")

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


if __name__ == "__main__":
    
    curZoneTaskThread = threading.Thread(인자들)
    
    objInfoTaskThread = threading.Thread(인자들)
    
    mainTaskThread = threading.Thread(인자들)
    
    curZoneTaskThread.start()
    objInfoTaskThread.start()
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
