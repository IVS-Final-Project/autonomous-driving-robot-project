#------globalVar.py-----------
zoneInfo = 0

# Variables shared for AI detection results
isObjDetected = False   # Flag: Person detected (Target to follow)
isObjInRoad = False     # Flag: Obstacle detected (Collision risk)
aiSteeringAngle = 0     # Steering angle calculated by AI

# Control Target Values
desiredSpeed = 0
desiredAngle = 0
