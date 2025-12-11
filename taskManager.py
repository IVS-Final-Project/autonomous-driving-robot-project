import globalVar
from time import sleep
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import sys
import gi
import threading
from hailoManager import HailoDetector

#=======================================================================
# HailoManager 불러오기
try:
    from hailoManager import HailoDetector
    hailo_available = True
except ImportError:
    print("[Warning] hailoManager not found. AI disabled.")
    hailo_available = False
#=======================================================================
# Hailo Detector 초기화 (에러 나도 통과)
hailo_detector = None
if hailo_available:
    try:
        HEF_PATH = "/usr/local/hailo/resources/models/hailo8/yolov8m.hef" # 수정해야 돼
        hailo_detector = HailoDetector(HEF_PATH)
    except Exception as e:
        print(f"[Warning] Hailo Init Failed: {e}")
#=======================================================================
# Mapping table: ArUco ID zone name
ZONE_MAP = {
    0: "Normal Zone",
    1: "Children Protection Zone",
    2: "Accident-Prone Zone",
    3: "Bump Zone",
}
#=======================================================================
# 속도 값
vel_ChildZone= 30 # 어린이 보호구역 목표 속도
vel_HighAccidentZone = 50 # 사고다발지역 목표 속도
vel_SpeedBump = 20 # 과속 방지턱 목표 속도
vel_ObjInRoad = 10 # 도로 밖에서 장애물을 인식했을 때 목표 속도
#=======================================================================
# 카메라 접근 충돌 방지
camera_lock = threading.Lock()
#=======================================================================
# Picamera Init
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (320, 240), "format": "BGR888"}
)
picam2.configure(config)
picam2.start()
time.sleep(1)
#=======================================================================
# Load ArUco dictionary (4x4_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters_create()
#=======================================================================
# Hailo 설정값 -> 사람 빼고 다 버려도 되지 않나?
OBSTACLE_CLASSES = ['chair', 'couch', 'potted plant', 'bed', 'dining table', 'tv', 'suitcase', 'backpack', 'umbrella']
COLLISION_AREA_THRESHOLD = 0.15
#=======================================================================


# TASK
def getCurZoneTask(stop_event, args):
    while not stop_event.is_set():
        try:
            frame = None
            # 1. 카메라 Lock 걸고
            with camera_lock:
                if picam2 is not None:
                    frame = picam2.capture_array()
            
            # 2. ArUco 인식
            if frame is not None:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                globalVar.zoneInfo = getCurZone(frame_bgr)
            
            sleep(0.1)
        except Exception as e:
            # print(f"ZoneTask Error: {e}")
            sleep(1)

def getObjInfoTask(stop_event, args):
    while not stop_event.is_set():
        try:
            frame = None
            with camera_lock:
                if picam2 is not None:
                    frame = picam2.capture_array()

            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                globalVar.isObjDetected, globalVar.isObjInRoad = getObjInfo(frame)

            sleep(0.1)

        except Exception as e:
            print(f"ObjTask Error: {e}")
            sleep(1)

def mainTask(stop_event, args):
    while not stop_event.is_set():
        zone = globalVar.zoneInfo
        detected = globalVar.isObjDetected
        inRoad = globalVar.isObjInRoad
        speed = 10
        
        if detected:
            if inRoad:
                speed = 0
            else:
                speed = vel_ObjInRoad
        
        elif zone == 0:
            speed = globalVar.desiredSpeed
        elif zone == 1:
            speed = vel_ChildZone
        elif zone == 2:
            speed = vel_HighAccidentZone
        elif zone == 3:
            speed = vel_SpeedBump
        
        globalVar.desiredSpeed = speed

        print("=====================================")
        print("desiredSpeed: ",globalVar.desiredSpeed)
        print("=====================================")
        sleep(0.1)

def getCurZone(frame):
    """
    - 입력: 카메라 프레임 (frame)
    - 출력: 감지된 Zone ID (없으면 -1)
    """

    if frame is None:
            return -1

    corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, aruco_dict, parameters=aruco_params
        )
    zone_id = -1  # Default: unmapped OR not detected
    zone_name = "Unknown Zone"

    if ids is not None:
        #cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            # corner = corners[i][0]
            # cx = int(np.mean(corner[:, 0]))
            # cy = int(np.mean(corner[:, 1]))

            if marker_id in ZONE_MAP:
                zone_id = marker_id
                zone_name = ZONE_MAP[marker_id]
            else:
                zone_id = -1
                zone_name = "Unknown Zone"

            # cv2.putText(
            #     frame,
            #     f"{zone_name} (ID:{zone_id})",
            #     (cx - 40, cy - 20),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.6,
            #     (0, 255, 0),
            #     2,
            # )

            print(f"Detected Marker ID: {marker_id} Zone ID: {zone_id} ({zone_name})")

            # for pt in corner:
            #     x, y = int(pt[0]), int(pt[1])
            #     cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
    
    # cv2.imshow("ArUco Zone Detection", frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    # cv2.destroyAllWindows()
    return zone_id


#=======================================================================
# Using Hailo
#=======================================================================
def getObjDetected(frame):
    """
    Hailo를 이용해 객체(사람) 감지
    """
    isDetected = 0
    bbox = None
    
    # Hailo가 없거나 초기화 실패 시 빈 리스트 반환
    detections = []
    if hailo_detector and hailo_detector.active:
        detections = hailo_detector.infer(frame)
    
    h, w, _ = frame.shape
    priority_found = False
    max_area = 0
    target_box = None # [x1, y1, x2, y2]

    for det in detections:
        label = det['label']
        # 0.0~1.0 정규화 좌표 -> 픽셀 좌표 변환
        ymin, xmin, ymax, xmax = det['bbox']
        x1, y1, x2, y2 = int(xmin*w), int(ymin*h), int(xmax*w), int(ymax*h)
        area = (y2 - y1) * (x2 - x1)

        # Logic A: 장애물 (우선순위 1)
        if label in OBSTACLE_CLASSES:
            if area > (w * h * COLLISION_AREA_THRESHOLD):
                priority_found = True
                target_box = [x1, y1, x2, y2]
                isDetected = 1
                break # 장애물 발견 시 즉시 확정

        # Logic B: 사람 (우선순위 2 - 장애물 없을 때)
        if not priority_found and label == 'person':
            if area > max_area:
                max_area = area
                target_box = [x1, y1, x2, y2]
                isDetected = 1
    return isDetected, target_box

def getObjInRoad(bbox, lines, frame_shape):
    isObjInRoad = 0
    if len(lines) < 2:
        return isObjInRoad

    h, w = frame_shape[:2]
    x1, y1, x2, y2 = bbox

    cx = (x1 + x2) / 2.0
    cy = float(y2)

    xs = []
    for (a, b, c) in lines:
        if abs(a) < 1e-6:
            continue
        x_on_line = -(b * cy + c) / a
        xs.append(x_on_line)

    if len(xs) < 2:
        return isObjInRoad

    xs.sort()
    left_x = xs[0]
    right_x = xs[-1]

    if left_x <= cx <= right_x:
        isObjInRoad = 1
    else:
        isObjInRoad = 0

    return isObjInRoad

def getObjInfo(frame):
    """
    프레임 -> 라인 추출 -> 객체(사람) 인식 & 도로 위 판별
    """
    h, w = frame.shape[:2]
    red_mask = get_red_mask(frame)  # 빨간색 마스크 추출
    lines = fit_red_lines(red_mask) # 직선 방정식 추출  

    # 2. 사람 감지
    isObjDetected, bbox = getObjDetected(frame)   

    # 3. 도로 위 판별
    isObjInRoad = 0
    if isObjDetected and bbox is not None:
        isObjInRoad = getObjInRoad(bbox, lines, (h, w))
    
    # print("=====================================")
    # print("isObjDetected: ", isObjDetected)
    # print("=====================================")

    return isObjDetected, isObjInRoad

#=======================================================================
# getObjInRoad 보조 함수들
#=======================================================================
def get_red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower1, upper1 = np.array([0, 100, 100]), np.array([10, 255, 255])
    lower2, upper2 = np.array([170, 100, 100]), np.array([180, 255, 255])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2), cv2.MORPH_OPEN, kernel, iterations=1)

def fit_red_lines(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts: return []
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:2]
    lines = []
    for c in cnts:
        try:
            vx, vy, x0, y0 = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            lines.append((float(vy), float(-vx), float(vx*y0 - vy*x0)))
        except: pass
    return lines