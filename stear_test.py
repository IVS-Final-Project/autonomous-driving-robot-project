import time
import globalVar
import cv2
from motorControl import lonControl, latControl
from roadInOutTF import get_red_mask, fit_red_curves, get_lane_center_error
from picamera2 import Picamera2

def create_camera():
    # Picamera2가 설치되어 있고 사용 가능하면 그것을 쓰고,
    # 아니면 OpenCV VideoCapture(웹캠)로 폴백합니다.
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 360), "format": "BGR888"})
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)
        return ("picam2", picam2)
    except Exception:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        return ("cv2", cap)

def lane_following_control_loop():
    cam_type, cam = create_camera()
    kp = 0.1  # 픽셀->각도 비례 이득 (환경에 맞게 튜닝)
    target_speed = 0

    try:
        while True:
            # 1) 프레임 캡처
            if cam_type == "picam2":
                frame = cam.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cam.read()
                if not ret:
                    print("Camera read failed")
                    break

            # 2) 차선(빨간) 마스크 및 곡선 피팅 (roadInOutTF의 함수 사용)
            red_mask = get_red_mask(frame)
            curves = fit_red_curves(red_mask)  # 파일에 정의되어 있어야 함

            # 3) lane center error 계산
            result = get_lane_center_error(curves, frame.shape, y_ref_ratio=0.9)
            if result is None:
                error_px, lane_x, cam_x = (None, None, None)
            else:
                error_px, lane_x, cam_x = result

            # 4) 단순 P 제어로 각도 결정
            if error_px is None:
                angle = 0.0
            else:
                angle = kp * error_px
                # 각도 범위(예시): -90..90
                angle = max(-90.0, min(90.0, angle))

            # 5) 속도/조향 적용
            globalVar.desiredSpeed = target_speed
            globalVar.desiredAngle = angle
            lonControl(globalVar.desiredSpeed)
            latControl(globalVar.desiredAngle)

            # 6) 디버그 출력 & 시각화 간단 표시
            print(f"error_px={error_px}, angle={angle:.2f}")

            # 프레임 표시 (선택)
            debug_vis = frame.copy()
            cv2.imshow("Frame", debug_vis)
            cv2.imshow("Red Mask", red_mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        if cam_type == "picam2":
            cam.stop()
        else:
            cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    lane_following_control_loop()