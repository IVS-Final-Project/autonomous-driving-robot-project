import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Mapping table: ArUco ID -> zone name
ZONE_MAP = {
    0: "Normal Zone",
    1: "Children Protection Zone",
    2: "Accident-Prone Zone",
    3: "Bump Zone",
}

def live_aruco_detection():
    """
    Detect ArUco markers continuously.
    - If ID is mapped return mapped ID
    - If ID is not mapped return -1
    - DO NOT stop detection loop when a marker is detected
    """

    # 1. Load ArUco dictionary (4x4_250)
    # Note: Use getPredefinedDictionary for newer OpenCV versions
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    
    # 2. Setup Parameters (Fixed for OpenCV 4.7+)
    # Old: cv2.aruco.DetectorParameters_create() -> REMOVED
    # New: cv2.aruco.DetectorParameters()
    aruco_params = cv2.aruco.DetectorParameters()

    # 3. Create Detector Object (Required for OpenCV 4.7+)
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Initialize Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "BGR888"} # Increased resolution slightly for better detection
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    print("ArUco Detector Started. Press 'q' to exit.")

    while True:
        # Capture frame
        frame = picam2.capture_array()
        
        # Picamera2 captures in BGR by default with "BGR888" format, 
        # but sometimes explicit conversion ensures compatibility
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) 

        # 4. Detect markers using the detector object
        # Old: cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        # New: detector.detectMarkers(frame)
        corners, ids, rejected = detector.detectMarkers(frame)

        zone_id = -1  # Default: unmapped OR not detected
        zone_name = "Unknown Zone"

        if ids is not None:
            # Draw markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                # Get corner points of the current marker
                corner = corners[i][0]
                
                # Calculate center point
                cx = int(np.mean(corner[:, 0]))
                cy = int(np.mean(corner[:, 1]))

                # ---- Determine zone ID ----
                if marker_id in ZONE_MAP:
                    zone_id = marker_id
                    zone_name = ZONE_MAP[marker_id]
                else:
                    zone_id = -1
                    zone_name = "Unknown Zone"

                # Display zone name near the marker
                cv2.putText(
                    frame,
                    f"{zone_name} (ID:{zone_id})",
                    (cx - 40, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                print(f"Detected Marker ID: {marker_id} -> Zone: {zone_name}")

                # Draw corner points for visual verification
                for pt in corner:
                    x, y = int(pt[0]), int(pt[1])
                    cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
        

        # Show frame
        cv2.imshow("ArUco Zone Detection", frame)

        # Quit loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()


def main():
    print("Starting ArUco Zone Detection...")
    live_aruco_detection()
    print("Camera stopped.")

if __name__ == "__main__":
    main()
