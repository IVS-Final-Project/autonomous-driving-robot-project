#-----taskManager.py--------
import globalVar
from time import sleep
import sys
import gi
import hailo

# Load GStreamer library
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# --- [Configuration Constants] ---
zoneID = {'IDLE' : 0, 'CHILD' : 1, 'HIGHACCIDENT' : 2, 'SPEEDBUMP' : 3}

# Target speeds for each zone
vel_ChildZone = 30 
vel_HighAccidentZone = 50 
vel_SpeedBump = 20 
vel_ObjDetected = 40    # Speed when following a person

# Collision Avoidance Settings
OBSTACLE_CLASSES = ['chair', 'couch', 'potted plant', 'bed', 'dining table', 'tv', 'suitcase', 'backpack', 'umbrella']
COLLISION_AREA_THRESHOLD = 0.15 
COLLISION_X_MIN = 0.3
COLLISION_X_MAX = 0.7
STEERING_GAIN = 160.0  # Steering sensitivity

# --- [GStreamer Callback Function] ---
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    person_found = False
    obstacle_found = False
    target_angle = 0

    # Variable to store the closest person
    best_person = None

    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        
        center_x = (bbox.xmin() + bbox.xmax()) / 2
        area = (bbox.ymax() - bbox.ymin()) * (bbox.xmax() - bbox.xmin())

        # 1. Obstacle Detection (Collision Avoidance)
        if label in OBSTACLE_CLASSES:
            # Check if the obstacle is large enough and in the center path
            if (area > COLLISION_AREA_THRESHOLD) and (COLLISION_X_MIN < center_x < COLLISION_X_MAX):
                obstacle_found = True
                # Priority 1: Stop immediately if obstacle is found
                break 

        # 2. Person Detection (Following)
        if label == "person":
            # If multiple people are detected, track the largest (closest) one
            if best_person is None or area > best_person['area']:
                best_person = {'center_x': center_x, 'area': area}

    # --- [Update globalVar] ---
    if obstacle_found:
        globalVar.isObjInRoad = True
        globalVar.isObjDetected = False # Stop following if collision risk exists
    elif best_person:
        globalVar.isObjInRoad = False
        globalVar.isObjDetected = True
        
        # Calculate Steering Angle
        error_x = best_person['center_x'] - 0.5
        steering_angle = error_x * STEERING_GAIN
        
        # Limit Angle (-90 to 90)
        if steering_angle > 90: steering_angle = 90
        if steering_angle < -90: steering_angle = -90
        
        globalVar.aiSteeringAngle = steering_angle
    else:
        # Reset if nothing is detected
        globalVar.isObjInRoad = False
        globalVar.isObjDetected = False
        globalVar.aiSteeringAngle = 0

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

# [Modified] Thread running the Hailo AI Loop
def getObjInfoTask(stop_event, args):
    print("Starting Hailo AI Task...")
    Gst.init(None)
    pipeline_string = get_pipeline_string()
    
    try:
        pipeline = Gst.parse_launch(pipeline_string)
    except Exception as e:
        print(f"GStreamer Error: {e}")
        return

    # Attach Callback
    # Important: Probe 'display_input_q' to get processed data
    hailo_filter = pipeline.get_by_name("display_input_q")
    hailo_filter_pad = hailo_filter.get_static_pad("src")
    hailo_filter_pad.add_probe(Gst.PadProbeType.BUFFER, app_callback, None)

    pipeline.set_state(Gst.State.PLAYING)
    
    # GStreamer MainLoop (Blocking)
    # Since this is a separate thread, it won't block the mainTask
    loop = GLib.MainLoop()
    
    try:
        loop.run()
    except Exception:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        
def mainTask(stop_event, arg):
    while not stop_event.is_set():
        zone = globalVar.zoneInfo
        detected = globalVar.isObjDetected
        inRoad = globalVar.isObjInRoad
        
        target_speed = 0
        target_angle = 0
        
        # 1. Obstacle Detected (Priority: Emergency Stop)
        if inRoad:
            target_speed = 0
            target_angle = 0
            print("Status: [OBSTACLE] Emergency Stop")
            
        # 2. Person Detected (Mode: Following)
        elif detected:
            target_speed = vel_ObjDetected
            target_angle = globalVar.aiSteeringAngle # Apply angle calculated by AI
            print(f"Status: [FOLLOW] Angle: {target_angle:.1f}")

        # 3. Normal Driving Mode (Zone-based speed control)
        else:
            target_angle = 0 # Go Straight (Or add line tracing logic here)
            
            if zone == zoneID['IDLE']:
                target_speed = 0 # Or default speed
            elif zone == zoneID['CHILD']:
                target_speed = vel_ChildZone
            elif zone == zoneID['HIGHACCIDENT']:
                target_speed = vel_HighAccidentZone
            elif zone == zoneID['SPEEDBUMP']:
                target_speed = vel_SpeedBump
        
        # Update globalVar -> motorControl will read this
        globalVar.desiredSpeed = target_speed
        globalVar.desiredAngle = target_angle
        
        sleep(0.1)

# (Dummy functions - Implement actual sensor logic later)
def getCurZone(arg):
    # Returning 0 (IDLE) for testing. Connect QR code logic here.
    return 0
    
    
