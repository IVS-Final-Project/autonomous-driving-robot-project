import cv2
import numpy as np
from picamera2 import Picamera2
from motorControl import lonControl, latControl

# ==========================================
# [移대찓???ㅼ젙] 鍮?諛섏궗 ?듭젣 紐⑤뱶
# ==========================================
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "BGR888"}, 
    controls={
        "FrameDurationLimits": (33333, 33333),
        "AeEnable": False,      # ?먮룞 ?몄텧 ??        "ExposureTime": 10000,  # 10ms (?대몼寃??댁꽌 鍮쏅컲??以꾩엫)
        "AnalogueGain": 1.0     # 寃뚯씤 ??땄 (?몄씠利?媛먯냼)
    }
)
picam2.configure(config)
picam2.start()

print("Line Tracing Started. Press ESC to exit.")

# ?붾㈃ 以묒븰 (紐⑺몴 吏??
CENTER_X = 320 

STEERING_SENSITIVITY = 0.3

try:
    while True:
        image = picam2.capture_array()

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 80]) # V 媛믪쓣 議곗젙?섏옄 
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        kernel = np.ones((9, 9), np.uint8) # ??媛믩룄 ?쒕떇 ?꾩슂?
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        h, w = mask.shape
        roi_h = 300 # ?쒕떇 ?꾩슂
        roi = mask[roi_h:h, 0:w]
        M = cv2.moments(roi)
        
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00']) + roi_h
            
            # [?쒖뼱 濡쒖쭅]
            # ?쇱씤???쇱そ(100)???덉쑝硫? 100 - 320 = -220 -> ?뚯닔(Left)
            # ?쇱씤???ㅻⅨ履?500)???덉쑝硫? 500 - 320 = +180 -> ?묒닔(Right)
            pixel_error = cx - CENTER_X
            
            # 議고뼢媛?怨꾩궛
            steering_angle = pixel_error * STEERING_SENSITIVITY
            steering_angle = 0
            
            # 紐⑦꽣 ?쒖뼱 ?⑥닔 ?몄텧
            lonControl(0)
            latControl(steering_angle)
            
            # [?쒓컖??
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1) # 鍮④컙?? ?쇱씤
            cv2.line(image, (CENTER_X, cy), (cx, cy), (0, 255, 0), 2) # 珥덈줉?? ?먮윭
            cv2.putText(image, f"Angle: {int(steering_angle)}", (10, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        else:
            print("Line Lost!")

        cv2.line(image, (CENTER_X, 0), (CENTER_X, 480), (255, 0, 0), 1)
        cv2.imshow("Cam", image)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cv2.destroyAllWindows()
    picam2.stop()
