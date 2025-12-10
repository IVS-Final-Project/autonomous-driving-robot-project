#------globalVar.py-----------
zoneInfo = 0

# [NEW] ArUco ID -> Zone ID Mapping
# (ID: 0=IDLE, 1=CHILD, 2=ACCIDENT, 3=BUMP)

ZONE_MAP = {
    0: 0,  # Normal
    1: 1,  # Child Zone
    2: 2,  # Accident Zone
    3: 3   # Speed Bump
}

# AI Detection Flags
isObjDetected = False
isObjInRoad = False
aiSteeringAngle = 0

# Control Targets
desiredSpeed = 0
desiredAngle = 0
