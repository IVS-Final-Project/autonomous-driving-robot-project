import globalVar
from time import sleep
import sys
import numpy as np
import cv2
import gi
import hailo
import logging
import config
from motorControl import lonControl, latControl

# 로깅 설정
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL, logging.INFO),
    format='%(asctime)s - [%(levelname)s] - %(message)s'
)
logger = logging.getLogger(__name__)

# GStreamer 라이브러리 로드
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# --- [설정 상수] ---
zoneID = config.ZONE_IDS
vel_IDLE = config.VELOCITY_IDLE
vel_ChildZone = config.VELOCITY_CHILD_ZONE
vel_HighAccidentZone = config.VELOCITY_HIGH_ACCIDENT_ZONE
vel_SpeedBump = config.VELOCITY_SPEED_BUMP
vel_ObjInRoad = config.VELOCITY_OBJ_IN_ROAD

# ArUco 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters_create()
ZONE_MAP = config.ZONE_MAP

# Hailo 장애물 클래스
OBSTACLE_CLASSES = config.OBSTACLE_CLASSES
COLLISION_AREA_THRESHOLD = config.COLLISION_AREA_THRESHOLD

# --------------------------------------------------------------------------
# [보조 함수] OpenCV 로직 - 예외 처리 추가
# --------------------------------------------------------------------------
def get_red_mask(frame):
    """빨간 라인 마스크 생성"""
    try:
        if frame is None or frame.size == 0:
            logger.warning("Invalid frame in get_red_mask")
            return None
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV) 
        lower1, upper1 = np.array([0, 100, 100]), np.array([10, 255, 255])
        lower2, upper2 = np.array([170, 100, 100]), np.array([180, 255, 255])
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2), cv2.MORPH_OPEN, kernel, iterations=1)
    except Exception as e:
        logger.error(f"Error in get_red_mask: {e}")
        return None

def fit_red_lines(mask):
    """빨간 라인을 직선으로 피팅"""
    try:
        if mask is None or mask.size == 0:
            logger.warning("Invalid mask in fit_red_lines")
            return []
        
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cnts:
            return []
        
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:2]
        lines = []
        for c in cnts:
            try:
                vx, vy, x0, y0 = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
                lines.append((float(vy), float(-vx), float(vx*y0 - vy*x0)))
            except Exception as e:
                logger.debug(f"Error fitting line to contour: {e}")
                pass
        return lines
    except Exception as e:
        logger.error(f"Error in fit_red_lines: {e}")
        return []

def check_obj_in_road(bbox, lines, frame_w, frame_h):
    """객체가 도로 위에 있는지 판별"""
    try:
        if bbox is None or not lines or len(lines) < 2:
            return False
        
        ymin, xmin, ymax, xmax = bbox
        x1, y1 = xmin * frame_w, ymin * frame_h
        x2, y2 = xmax * frame_w, ymax * frame_h
        cx = (x1 + x2) / 2.0
        cy = float(y2)
        
        xs = []
        for (a, b, c) in lines:
            if abs(a) < 1e-6:
                continue
            x_on_line = -(b * cy + c) / a
            xs.append(x_on_line)
        
        if len(xs) < 2:
            return False
        
        xs.sort()
        if xs[0] <= cx <= xs[-1]:
            return True
        return False
    except Exception as e:
        logger.error(f"Error in check_obj_in_road: {e}")
        return False

# --------------------------------------------------------------------------
# [메인] GStreamer 콜백 함수 - 예외 처리 추가
# --------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    try:
        buffer = info.get_buffer()
        if buffer is None:
            logger.debug("Buffer is None")
            return Gst.PadProbeReturn.OK

        # [중요] READ 전용으로 맵핑 (에러 원인 제거됨)
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            logger.warning("Failed to map buffer")
            return Gst.PadProbeReturn.OK
        
        # 원본 메모리 참조
        try:
            frame = np.ndarray(
                shape=(640, 640, 3),
                dtype=np.uint8,
                buffer=map_info.data
            )
            
            if frame is None or frame.size == 0:
                logger.warning("Invalid frame in callback")
                buffer.unmap(map_info)
                return Gst.PadProbeReturn.OK
            
            # ---------------- [A. ArUco 인식] ----------------
            try:
                corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
                current_zone = -1
                if ids is not None:
                    for marker_id in ids.flatten():
                        if marker_id in ZONE_MAP:
                            current_zone = marker_id
                            break
                
                if current_zone != -1:
                    globalVar.zoneInfo = current_zone
            except Exception as e:
                logger.warning(f"Error in ArUco detection: {e}")

            # ---------------- [B. Hailo 객체 인식] ----------------
            try:
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
                    if red_mask is not None:
                        lines = fit_red_lines(red_mask)
                        in_road = check_obj_in_road(bbox, lines, 640, 640)
                    
                globalVar.isObjInRoad = in_road
            except Exception as e:
                logger.error(f"Error in Hailo object detection: {e}")
                globalVar.isObjDetected = False
                globalVar.isObjInRoad = False

        finally:
            # 맵핑 해제
            buffer.unmap(map_info)
        
        return Gst.PadProbeReturn.OK
    
    except Exception as e:
        logger.error(f"Fatal error in app_callback: {e}")
        return Gst.PadProbeReturn.OK

# --------------------------------------------------------------------------
# 파이프라인 구성
# --------------------------------------------------------------------------
def get_pipeline_string():
    """GStreamer 파이프라인 문자열 생성"""
    hef_path = config.HAILO_HEF_PATH
    post_process_so = config.HAILO_POST_PROCESS_SO
    
    pipeline = (
        "libcamerasrc ! video/x-raw, format=" + config.CAMERA_INPUT_FORMAT + ", width=" + str(config.CAMERA_INPUT_WIDTH) + ", height=" + str(config.CAMERA_INPUT_HEIGHT) + " ! "
        "videoconvert ! "
        "videoscale ! video/x-raw, format=" + config.CAMERA_INPUT_FORMAT + ", width=" + str(config.INFERENCE_WIDTH) + ", height=" + str(config.INFERENCE_HEIGHT) + " ! "
        "queue name=inference_input_q max-size-buffers=" + str(config.QUEUE_MAX_BUFFERS) + " ! "
        "hailonet hef-path=" + hef_path + " batch-size=1 ! "
        "queue name=inference_output_q max-size-buffers=" + str(config.QUEUE_MAX_BUFFERS) + " ! "
        "hailofilter so-path=" + post_process_so + " function-name=filter_letterbox qos=false ! "
        "queue name=hailo_post_q ! " 
        "hailooverlay ! "
        "videoconvert ! "
        "video/x-raw, format=" + config.CAMERA_INPUT_FORMAT + " ! " 
        "queue name=draw_q ! "      
        "videoconvert ! "          
        "fpsdisplaysink video-sink=autovideosink text-overlay=false sync=false" 
    )
    return pipeline

# --------------------------------------------------------------------------
# [Task] 실행 함수 - 예외 처리 추가
# --------------------------------------------------------------------------
def getObjInfoTask(stop_event, args):
    """GStreamer 통합 태스크"""
    logger.info("Starting GStreamer Integrated Task...")
    try:
        Gst.init(None)
        pipeline_string = get_pipeline_string()
        
        try:
            pipeline = Gst.parse_launch(pipeline_string)
        except Exception as e:
            logger.error(f"GStreamer pipeline parsing error: {e}")
            return

        # draw_q에 프로브를 연결
        try:
            target_element = pipeline.get_by_name("draw_q")
            if target_element:
                pad = target_element.get_static_pad("src")
                if pad:
                    pad.add_probe(Gst.PadProbeType.BUFFER, app_callback, None)
                    logger.info("Probe attached successfully")
                else:
                    logger.error("Could not get src pad from draw_q")
            else:
                logger.error("Could not find pipeline element 'draw_q'")
                return
        except Exception as e:
            logger.error(f"Error attaching probe: {e}")
            return

        try:
            pipeline.set_state(Gst.State.PLAYING)
            logger.info("GStreamer pipeline started")
        except Exception as e:
            logger.error(f"Error starting pipeline: {e}")
            return
        
        loop = GLib.MainLoop()
        
        try:
            while not stop_event.is_set():
                try:
                    GLib.MainContext.default().iteration(False)
                    sleep(config.GSTREAMER_ITERATION_INTERVAL)
                except Exception as e:
                    logger.error(f"Error in main loop iteration: {e}")
                    break
        except KeyboardInterrupt:
            logger.info("Interrupted")
        except Exception as e:
            logger.error(f"Fatal error in getObjInfoTask: {e}")
        finally:
            try:
                pipeline.set_state(Gst.State.NULL)
                logger.info("Pipeline stopped")
            except Exception as e:
                logger.error(f"Error stopping pipeline: {e}")
    
    except Exception as e:
        logger.error(f"Fatal error in getObjInfoTask: {e}")

def getCurZoneTask(stop_event, args):
    """현재 구역 확인 태스크"""
    try:
        while not stop_event.is_set():
            try:
                sleep(config.ZONE_CHECK_INTERVAL)
            except Exception as e:
                logger.error(f"Error in getCurZoneTask: {e}")
                break
    except Exception as e:
        logger.error(f"Fatal error in getCurZoneTask: {e}")

def mainTask(stop_event, args):
    """메인 제어 루프"""
    try:
        while not stop_event.is_set():
            try:
                # 1. 센서 정보
                zone = globalVar.zoneInfo
                detected = globalVar.isObjDetected
                inRoad = globalVar.isObjInRoad
                target_speed = globalVar.userTargetSpeed
                
                # 2. [핵심] 속도 결정 로직 (Set)
                final_speed = target_speed  # 기본값
                
                # (A) 평소(IDLE) -> 내 키보드 속도(target_speed)를 따름
                if zone == zoneID['IDLE']:
                    final_speed = target_speed
                    logger.debug(f"[IDLE] Manual Control: {final_speed}")

                # (B) 특정 구역 -> 해당 속도로 강제 고정 (Set)
                elif zone == zoneID['CHILD']: 
                    final_speed = vel_ChildZone  # 설정값으로 고정
                    logger.info(f"[CHILD] Speed Fixed to {final_speed}")
                    
                elif zone == zoneID['HIGHACCIDENT']:
                    final_speed = vel_HighAccidentZone # 설정값으로 고정
                    logger.info(f"[HIGHACCIDENT] Speed Fixed to {final_speed}")
                    
                elif zone == zoneID['SPEEDBUMP']:
                    final_speed = vel_SpeedBump  # 설정값으로 고정
                    logger.info(f"[SPEEDBUMP] Speed Fixed to {final_speed}")
                    
                else:
                    logger.debug(f"Unknown zone: {zone}")

                # 3. [최우선] 장애물 감지 시 정지 (0으로 덮어쓰기)
                if detected:
                    if inRoad:
                        final_speed = 0
                        logger.warning(f"[STOP] Obstacle in Road!")

                # 4. 결과 업데이트
                try:
                    globalVar.desiredSpeed = final_speed
                    lonControl(final_speed)
                except Exception as e:
                    logger.error(f"Error controlling motor: {e}")
                
                sleep(config.MAIN_LOOP_INTERVAL)
            
            except Exception as e:
                logger.error(f"Error in main control loop iteration: {e}")
                sleep(config.RETRY_DELAY)
    
    except Exception as e:
        logger.error(f"Fatal error in mainTask: {e}")
