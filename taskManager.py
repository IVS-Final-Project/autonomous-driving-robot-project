#-----taskManager.py--------
import globalVar
from time import sleep
import sys
import gi
import hailo
import threading
import cv2
import numpy as np

# Load GStreamer library
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# --- [Configuration Constants] ---
zoneID = {'IDLE' : 0, 'CHILD' : 1, 'HIGHACCIDENT' : 2, 'SPEEDBUMP' : 3}

# Target speeds
vel_Cruising = 40
vel_ChildZone = 30 
vel_HighAccidentZone = 50 
vel_SpeedBump = 20 

# Collision Settings
OBSTACLE_CLASSES = ['chair', 'couch', 'potted plant', 'bed', 'dining table', 'tv', 'suitcase', 'backpack', 'umbrella']
OBSTACLE_STOP_AREA = 0.10
PERSON_STOP_AREA = 0.12
COLLISION_X_MIN = 0.3
COLLISION_X_MAX = 0.7

# --- [ArUco Setup] ---
# 1. Load Dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
# 2. Setup Parameters
aruco_params = cv2.aruco.DetectorParameters()
# 3. Create Detector
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Frame Counter for optimization
frame_count = 0

# --- [GStreamer Callback Function] ---
def app_callback(pad, info, user_data):
    global frame_count
    
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # 1. Get Hailo Object Detection Results (Existing Logic)
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    stop_signal = False 

    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        
        center_x = (bbox.xmin() + bbox.xmax()) / 2
        area = (bbox.ymax() - bbox.ymin()) * (bbox.xmax() - bbox.xmin())

        if (COLLISION_X_MIN < center_x < COLLISION_X_MAX):
            if label in OBSTACLE_CLASSES and area > OBSTACLE_STOP_AREA:
                stop_signal = True
                break 
            if label == "person" and area > PERSON_STOP_AREA:
                stop_signal = True
                break

    # Update Object Status
    if stop_signal:
        globalVar.isObjInRoad = True
    else:
        globalVar.isObjInRoad = False

    # ---------------------------------------------------------
    # 2. ArUco Marker Detection (Integrated Logic)
    # ---------------------------------------------------------
    # Run ArUco detection every 5 frames to save CPU
    frame_count += 1
    if frame_count % 5 == 0:
        
        # Convert GStreamer Buffer to Numpy Array (Image)
        caps = pad.get_current_caps()
        w = caps.get_structure(0).get_value('width')
        h = caps.get_structure(0).get_value('height')
        
        # Map buffer to read data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        
        if success:
            try:
                # Create numpy array from raw data (RGB format)
                image = np.ndarray(shape=(h, w, 3), dtype=np.uint8, buffer=map_info.data)
                
                # Detect Markers
                corners, ids, rejected = aruco_detector.detectMarkers(image)
                
                if ids is not None:
                    # Update Zone based on the first detected marker
                    found_id = ids.flatten()[0]
                    
                    if found_id in globalVar.ZONE_MAP:
                        new_zone = globalVar.ZONE_MAP[found_id]
                        # Only print if zone changes
                        if globalVar.zoneInfo != new_zone:
                            print(f"[ZONE UPDATE] Marker ID: {found_id} -> Zone: {new_zone}")
                            globalVar.zoneInfo = new_zone
            except Exception as e:
                print(f"ArUco Error: {e}")
            finally:
                # Must unmap buffer
                buffer.unmap(map_info)

    return Gst.PadProbeReturn.OK

# --- [Pipeline Setup] ---
def get_pipeline_string():
    hef_path = "/usr/local/hailo/resources/models/hailo8/yolov8m.hef"
    post_process_so = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
    
    source_element = (
        "libcamerasrc ! video/x-raw, format=RGB, width=640, height=480 ! "
        "videoconvert ! "
        "videoscale ! video/x-raw, format=RGB, width=640, height=640"
    )
    
    pipeline_string = (
        f'{source_element} ! '
        f'queue name=inference_input_q max-size-buffers=3 ! '
        f'hailonet hef-path={hef_path} batch-size=1 ! '
        f'queue name=inference_output_q max-size-buffers=3 ! '
        f'hailofilter so-path={post_process_so} function-name=filter_letterbox qos=false ! '
        f'queue name=display_input_q max-size-buffers=3 ! '
        f'hailooverlay ! '
        f'videoconvert ! '
        f'fpsdisplaysink video-sink=autovideosink name=hailo_display sync=false text-overlay=false'
    )
    return pipeline_string

# --- [Task Functions] ---

def getCurZoneTask(stop_event, args):
    # This task is now handled inside getObjInfoTask (Callback)
    # Just keep it alive or empty
    while not stop_event.is_set():
        sleep(1)

def getObjInfoTask(stop_event, args):
    print("Starting AI Task (Hailo Object + ArUco Zone)...")
    
    Gst.init(None)
    pipeline_string = get_pipeline_string()
    
    try:
        pipeline = Gst.parse_launch(pipeline_string)
    except Exception as e:
        print(f"GStreamer Error: {e}")
        return

    hailo_filter = pipeline.get_by_name("display_input_q")
    hailo_filter_pad = hailo_filter.get_static_pad("src")
    hailo_filter_pad.add_probe(Gst.PadProbeType.BUFFER, app_callback, None)

    pipeline.set_state(Gst.State.PLAYING)
    loop = GLib.MainLoop()
    
    def check_stop():
        if stop_event.is_set():
            loop.quit()
            return False 
        return True 
    
    GLib.timeout_add(100, check_stop)
    
    try:
        loop.run()
    except Exception as e:
        print(f"Loop Error: {e}")
    finally:
        print("Stopping AI Pipeline...")
        pipeline.set_state(Gst.State.NULL)


def mainTask(stop_event, arg):
    while not stop_event.is_set():
        zone = globalVar.zoneInfo
        inRoad = globalVar.isObjInRoad 
        
        target_speed = 0
        target_angle = 0 
        
        # 1. Safety Check
        if inRoad:
            target_speed = 0
            # print("Status: [STOP] Obstacle Detected!")
            
        # 2. Cruising Mode (Zone Speed Control)
        else:
            target_speed = vel_Cruising # Default
            
            if zone == zoneID['CHILD']:
                target_speed = vel_ChildZone
                # print(f"Zone: CHILD ({target_speed})")
            elif zone == zoneID['HIGHACCIDENT']:
                target_speed = vel_HighAccidentZone
                # print(f"Zone: HIGH ACCIDENT ({target_speed})")
            elif zone == zoneID['SPEEDBUMP']:
                target_speed = vel_SpeedBump
                # print(f"Zone: SPEED BUMP ({target_speed})")
            else:
                # print(f"Zone: NORMAL ({target_speed})")
                pass
        
        globalVar.desiredSpeed = target_speed
        globalVar.desiredAngle = target_angle
        
        sleep(0.1)

# Dummy
def getCurZone(arg):
    return 0
