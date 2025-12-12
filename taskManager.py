import globalVar
from time import sleep
import sys
import numpy as np
import cv2
import gi
import hailo
from motorControl import lonControl, latControl

# GStreamer 라이브러리 로드
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# --- [설정 상수] ---
zoneID = {'IDLE' : 0, 'CHILD' : 1, 'HIGHACCIDENT' : 2, 'SPEEDBUMP' : 3}
vel_IDLE =30
vel_ChildZone = 13
vel_HighAccidentZone = 20
vel_SpeedBump = 13
vel_ObjInRoad = 10

# ArUco 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters_create()
ZONE_MAP = {0: "Normal", 1: "Child", 2: "Accident", 3: "Bump"}

# Hailo 장애물 클래스
OBSTACLE_CLASSES = ['people', 'car']
COLLISION_AREA_THRESHOLD = 0.15

# --------------------------------------------------------------------------
# [보조 함수] OpenCV 로직
# --------------------------------------------------------------------------
def get_red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV) 
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

def check_obj_in_road(bbox, lines, frame_w, frame_h):
    if len(lines) < 2: return False
    ymin, xmin, ymax, xmax = bbox
    x1, y1 = xmin * frame_w, ymin * frame_h
    x2, y2 = xmax * frame_w, ymax * frame_h
    cx = (x1 + x2) / 2.0
    cy = float(y2)
    xs = []
    for (a, b, c) in lines:
        if abs(a) < 1e-6: continue
        x_on_line = -(b * cy + c) / a
        xs.append(x_on_line)
    if len(xs) < 2: return False
    xs.sort()
    if xs[0] <= cx <= xs[-1]:
        return True
    return False

# --------------------------------------------------------------------------
# [메인] GStreamer 콜백 함수 (로직만 수행, 그리기 X)
# --------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None: return Gst.PadProbeReturn.OK

    # [중요] READ 전용으로 맵핑 (에러 원인 제거됨)
    success, map_info = buffer.map(Gst.MapFlags.READ)
    if not success: return Gst.PadProbeReturn.OK
    
    # 원본 메모리 참조
    frame = np.ndarray(
        shape=(640, 640, 3),
        dtype=np.uint8,
        buffer=map_info.data
    )
    
    # ---------------- [A. ArUco 인식] ----------------
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    current_zone = -1
    if ids is not None:
        for marker_id in ids.flatten():
            if marker_id in ZONE_MAP:
                current_zone = marker_id
                break
    
    if current_zone != -1:
        globalVar.zoneInfo = current_zone

    # ---------------- [B. Hailo 객체 인식] ----------------
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    
    detected = False
    bbox = None
    priority_found = False
    max_area = 0

    for det in detections:
        label = det.get_label()
        b = det.get_bbox()
        area = (b.ymax() - b.ymin()) * (b.xmax() - b.xmin())
        
        if label in OBSTACLE_CLASSES:
            if area > COLLISION_AREA_THRESHOLD:
                detected = True
                priority_found = True
                bbox = [b.ymin(), b.xmin(), b.ymax(), b.xmax()]
                break 
        
        if not priority_found and label == 'person':
            if area > max_area:
                max_area = area
                detected = True
                bbox = [b.ymin(), b.xmin(), b.ymax(), b.xmax()]

    globalVar.isObjDetected = detected
    
    # ---------------- [C. 도로 위 판별] ----------------
    in_road = False
    if detected and bbox is not None:
        red_mask = get_red_mask(frame)
        lines = fit_red_lines(red_mask)
        in_road = check_obj_in_road(bbox, lines, 640, 640)
        
    globalVar.isObjInRoad = in_road

    # 맵핑 해제
    buffer.unmap(map_info)
    
    return Gst.PadProbeReturn.OK

# --------------------------------------------------------------------------
# 파이프라인 구성
# --------------------------------------------------------------------------
def get_pipeline_string():
    hef_path = "/usr/local/hailo/resources/models/hailo8/yolov8m.hef"
    post_process_so = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
    
    pipeline = (
        "libcamerasrc ! video/x-raw, format=RGB, width=640, height=480 ! "
        "videoconvert ! "
        "videoscale ! video/x-raw, format=RGB, width=640, height=640 ! "
        "queue name=inference_input_q max-size-buffers=3 ! "
        "hailonet hef-path=" + hef_path + " batch-size=1 ! "
        "queue name=inference_output_q max-size-buffers=3 ! "
        "hailofilter so-path=" + post_process_so + " function-name=filter_letterbox qos=false ! "
        "queue name=hailo_post_q ! " 
        "hailooverlay ! "
        "videoconvert ! "
        "video/x-raw, format=RGB ! " 
        "queue name=draw_q ! "      
        "videoconvert ! "          
        "fpsdisplaysink video-sink=autovideosink text-overlay=false sync=false" 
    )
    return pipeline

# --------------------------------------------------------------------------
# [Task] 실행 함수
# --------------------------------------------------------------------------
def getObjInfoTask(stop_event, args):
    print("Starting GStreamer Integrated Task...")
    Gst.init(None)
    pipeline_string = get_pipeline_string()
    
    try:
        pipeline = Gst.parse_launch(pipeline_string)
    except Exception as e:
        print(f"GStreamer Error: {e}")
        return

    # draw_q에 프로브를 연결
    target_element = pipeline.get_by_name("draw_q")
    if target_element:
        pad = target_element.get_static_pad("src")
        pad.add_probe(Gst.PadProbeType.BUFFER, app_callback, None)
    else:
        print("Error: Could not find pipeline element 'draw_q'")

    pipeline.set_state(Gst.State.PLAYING)
    loop = GLib.MainLoop()
    
    try:
        while not stop_event.is_set():
            GLib.MainContext.default().iteration(False)
            sleep(0.01) 
    except Exception as e:
        print(e)
    finally:
        pipeline.set_state(Gst.State.NULL)

def getCurZoneTask(stop_event, args):
    while not stop_event.is_set():
        sleep(1)

def mainTask(stop_event, args):
    while not stop_event.is_set():
        # 1. 센서 정보
        zone = globalVar.zoneInfo
        detected = globalVar.isObjDetected
        inRoad = globalVar.isObjInRoad
        target_speed = globalVar.userTargetSpeed
        
        # 2. [핵심] 속도 결정 로직 (Set)
        
        # (A) 평소(IDLE) -> 내 키보드 속도(target_speed)를 따름
        if zone == zoneID['IDLE']:
            final_speed = target_speed
            # print(f"[IDLE] Manual Control: {final_speed}")

        # (B) 특정 구역 -> 해당 속도로 강제 고정 (Set)
        elif zone == zoneID['CHILD']: 
            final_speed = vel_ChildZone  # 15로 고정
            print(f"[CHILD] Speed Fixed to {final_speed}")
            
        elif zone == zoneID['HIGHACCIDENT']:
            final_speed = vel_HighAccidentZone # 20으로 고정
            print(f"[HIGHACCIDENT] Speed Fixed to {final_speed}")
            
        elif zone == zoneID['SPEEDBUMP']:
            final_speed = vel_SpeedBump  # 25로 고정
            print(f"[SPEEDBUMP] Speed Fixed to {final_speed}")
            
        else:
            final_speed = target_speed # 예외 처리

        # 3. [최우선] 장애물 감지 시 정지 (0으로 덮어쓰기)
        if detected:
            if inRoad:
                final_speed = 0
                print(f"[STOP] Obstacle in Road!")

        # 4. 결과 업데이트
        globalVar.desiredSpeed = final_speed
        lonControl(final_speed)
        # (모터 제어는 main.py가 하므로 여기선 lonControl 호출 금지)
        
        sleep(0.1)
