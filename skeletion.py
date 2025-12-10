import threading
from time import sleep

#getCurZone
import cv2
import numpy as np
from picamera2 import Picamera2
import time

zoneID = {'IDLE' : 0, 'CHILD' : 1, 'HIGHACCIDENT' : 2, 'SPEEDBUMP' : 3}   

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

def getCurZone():
    """
    Detect ArUco markers continuously.
    - If ID is mapped return mapped ID
    - If ID is not mapped return -1
    - DO NOT stop detection loop when a marker is detected
    """

    # Load ArUco dictionary (4x4_250)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Initialize camera (Picamera2 on RPi, webcam on Windows)
    if USE_PICAMERA:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (320, 240), "format": "BGR888"}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1)
    else:
        # Windows: 웹캠 사용
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        time.sleep(1)

    while True:
        # Capture frame
        if USE_PICAMERA:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                print("웹캠을 읽을 수 없습니다")
                break
            frame = cv2.resize(frame, (320, 240))

        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, aruco_dict, parameters=aruco_params
        )

        zone_id = -1  # Default: unmapped OR not detected
        zone_name = "Unknown Zone"

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]
                cx = int(np.mean(corner[:, 0]))
                cy = int(np.mean(corner[:, 1]))

                # ---- Determine zone ID ----
                if marker_id in zoneID:
                    zone_id = marker_id
                    zone_name = zoneID[marker_id]
                else:
                    zone_id = -1
                    zone_name = "Unknown Zone"

                # Display zone name
                cv2.putText(
                    frame,
                    f"{zone_name} (ID:{zone_id})",
                    (cx - 40, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                print(f"Detected Marker ID: {marker_id} Zone ID: {zone_id} ({zone_name})")

                # Draw corner points
                for pt in corner:
                    x, y = int(pt[0]), int(pt[1])
                    cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
        

        # Show frame
        cv2.imshow("ArUco Zone Detection", frame)

        # DO NOT return: loop continues
        # Quit only if the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    if USE_PICAMERA:
        picam2.stop()
    else:
        cap.release()
    cv2.destroyAllWindows()


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
