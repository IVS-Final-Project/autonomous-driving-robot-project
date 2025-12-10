#-----taskManager.py--------
import globalVar
from time import sleep
import sys
import gi
import hailo
import threading

# Load GStreamer library
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# --- [Configuration Constants] ---
zoneID = {'IDLE' : 0, 'CHILD' : 1, 'HIGHACCIDENT' : 2, 'SPEEDBUMP' : 3}

# Target speeds
vel_Cruising = 40       # Default driving speed (Straight)
vel_ChildZone = 30 
vel_HighAccidentZone = 50 
vel_SpeedBump = 20 

# Collision & Safety Settings
OBSTACLE_CLASSES = ['chair', 'couch', 'potted plant', 'bed', 'dining table', 'tv', 'suitcase', 'backpack', 'umbrella']

# Thresholds for stopping
OBSTACLE_STOP_AREA = 0.10     # Stop if obstacle is bigger than 15%
PERSON_STOP_AREA = 0.005       # Stop if person is bigger than 20% (Too close)

# Region of Interest for Safety (Center of screen)
COLLISION_X_MIN = 0.3
COLLISION_X_MAX = 0.7

# --- [GStreamer Callback Function] ---
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    stop_signal = False # Flag to trigger emergency stop

    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        
        center_x = (bbox.xmin() + bbox.xmax()) / 2
        area = (bbox.ymax() - bbox.ymin()) * (bbox.xmax() - bbox.xmin())

        # Check if the object is in the driving path (Center)
        if (COLLISION_X_MIN < center_x < COLLISION_X_MAX):
            
            # Case 1: Inert Obstacles
            if label in OBSTACLE_CLASSES:
                if area > OBSTACLE_STOP_AREA:
                    stop_signal = True
                    break 

            # Case 2: Person (Now treated as a dynamic obstacle)
            if label == "person":
                if area > PERSON_STOP_AREA:
                    stop_signal = True
                    break

    # --- [Update globalVar] ---
    if stop_signal:
        globalVar.isObjInRoad = True # Danger detected
    else:
        globalVar.isObjInRoad = False # Path is clear

    return Gst.PadProbeReturn.OK

# --- [Create GStreamer Pipeline] ---
def get_pipeline_string():
    hef_path = "/usr/local/hailo/resources/models/hailo8/yolov8m.hef"
    post_process_so = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
    
    source_element = (
        "libcamerasrc ! video/x-raw, format=RGB, width=640, height=480 ! "
        "videoconvert ! "
        "videoscale ! video/x-raw, format=RGB, width=640, height=640"
    )
    
    # [FIXED] Changed glimagesink to autovideosink for better compatibility
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
    while not stop_event.is_set():
        globalVar.zoneInfo = getCurZone(args)
        sleep(0.1)

def getObjInfoTask(stop_event, args):
    print("Starting Hailo AI Task (Safety Mode)...")
    
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
    # [Safety] Wait a bit for AI to initialize
    sleep(2) 
    
    while not stop_event.is_set():
        zone = globalVar.zoneInfo
        inRoad = globalVar.isObjInRoad 
        
        target_speed = 0
        target_angle = 0 
        
        # 1. Safety Check (Priority: Highest)
        if inRoad:
            target_speed = 0
            print("Status: [STOP] Person or Obstacle Detected!")
            
        # 2. Cruising Mode (Go Straight)
        else:
            target_speed = vel_Cruising
            
            if zone == zoneID['CHILD']:
                target_speed = vel_ChildZone
            elif zone == zoneID['HIGHACCIDENT']:
                target_speed = vel_HighAccidentZone
            elif zone == zoneID['SPEEDBUMP']:
                target_speed = vel_SpeedBump
            
            # print(f"Status: [DRIVE] Speed: {target_speed}")
        
        globalVar.desiredSpeed = target_speed
        globalVar.desiredAngle = target_angle
        
        sleep(0.1)

def getCurZone(arg):
    return 0
