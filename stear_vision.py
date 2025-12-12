import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep
import logging
import config
import globalVar

# ===== 로깅 설정 =====
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL, logging.INFO),
    format='%(asctime)s - [%(levelname)s] - %(message)s'
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------
# 1. 라인 마스크 생성
# ---------------------------------------------------
def get_red_mask(frame):
    """
    프레임에서 빨간 라인을 감지하는 마스크 생성
    (입력 frame은 BGR 기준)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    red_mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    return red_mask


def fit_red_curves(red_mask):
    """
    red_mask: 단일 채널(0/255) 빨간 라인 마스크
    반환값:
        poly_list: 각 차선별 3차 다항식 계수 리스트
                   [a3, a2, a1, a0] 형태로
                   x = a3*y^3 + a2*y**2 + a1*y + a0
    디버깅:
        "Red Curve Fit Debug" 윈도우에
        - 컨투어 점(초록)
        - 피팅된 3차 곡선(파랑)
        을 시각화해서 보여줌.
    """
    try:
        if red_mask is None or red_mask.size == 0:
            raise ValueError("Invalid red_mask: mask is None or empty")
        
        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        if len(contours) < 1:
            logger.debug("No contours found in red mask")
            if config.DEBUG_VISUALIZATION_ENABLED:
                cv2.imshow("Red Curve Fit Debug", red_mask)
            return []

        # 큰 컨투어 2개까지만 사용 (좌/우 차선 가정)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        contours = contours[:2]

        h, w = red_mask.shape[:2]
        debug_vis = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)

        poly_list = []

        for cnt in contours:
            pts = cnt.reshape(-1, 2)
            xs = pts[:, 0].astype(np.float32)
            ys = pts[:, 1].astype(np.float32)

            if len(xs) < config.MIN_CONTOUR_POINTS:
                logger.debug(
                    f"Contour has less than {config.MIN_CONTOUR_POINTS} points: {len(xs)}"
                )
                continue

            try:
                # 3차 다항식 피팅: x = f(y)
                coeffs = np.polyfit(ys, xs, config.POLY_FIT_DEGREE)
                poly_list.append(coeffs)

                if config.DEBUG_VISUALIZATION_ENABLED:
                    # 컨투어 점 (초록)
                    for x, y in zip(xs.astype(int), ys.astype(int)):
                        if 0 <= x < w and 0 <= y < h:
                            cv2.circle(debug_vis, (x, y), 1, (0, 255, 0), -1)

                    # 피팅 곡선 (파랑)
                    y_min = int(np.clip(ys.min(), 0, h - 1))
                    y_max = int(np.clip(ys.max(), 0, h - 1))
                    if y_max <= y_min:
                        continue

                    y_samples = np.linspace(y_min, y_max, num=100).astype(np.float32)
                    x_samples = np.polyval(coeffs, y_samples)

                    prev_pt = None
                    for x_f, y_f in zip(x_samples, y_samples):
                        x_i = int(round(x_f))
                        y_i = int(round(y_f))
                        if 0 <= x_i < w and 0 <= y_i < h:
                            if prev_pt is not None:
                                cv2.line(debug_vis, prev_pt, (x_i, y_i), (255, 0, 0), 2)
                            prev_pt = (x_i, y_i)
                        else:
                            prev_pt = None

            except Exception as e:
                logger.warning(f"Error fitting curve to contour: {e}")
                continue

        if config.DEBUG_VISUALIZATION_ENABLED:
            cv2.imshow("Red Curve Fit Debug", debug_vis)
        
        logger.debug(f"Fitted {len(poly_list)} curves")
        return poly_list
    
    except Exception as e:
        logger.error(f"Error in fit_red_curves: {e}")
        raise


def get_lane_center_error(poly_list, frame_shape, y_ref_ratio=None):
    """
    poly_list: fit_red_curves(red_mask)가 반환한 3차 다항식 계수 리스트
               각 원소는 [a3, a2, a1, a0]
               x = a3*y^3 + a2*y**2 + a1*y + a0
    frame_shape: frame.shape  (h, w, c)
    y_ref_ratio: 화면 세로에서 어느 높이에서 기준을 잡을지 (0~1 비율)
                 None이면 config에서 기본값 사용

    반환:
        error_px       : lane_center_x - camera_center_x (픽셀 단위 오차)
        lane_center_x  : 기준 y에서 차선 중심 x 좌표
        camera_center_x: 화면 중심 x 좌표 (w/2)

    의미:
        error_px > 0 : 차선 중심이 카메라 기준 오른쪽에 있음
        error_px < 0 : 차선 중심이 카메라 기준 왼쪽에 있음
    """
    try:
        if y_ref_ratio is None:
            y_ref_ratio = config.LANE_Y_REF_RATIO
        
        if not (0 <= y_ref_ratio <= 1):
            raise ValueError(f"y_ref_ratio must be between 0 and 1, got {y_ref_ratio}")
        
        h, w = frame_shape[:2]

        y_ref = int(h * y_ref_ratio)
        y_ref = max(0, min(h - 1, y_ref))

        xs = []
        for coeffs in poly_list:
            try:
                if len(coeffs) != 4:
                    logger.warning(f"Invalid coefficient length: {len(coeffs)}")
                    continue
                
                x_val = float(np.polyval(coeffs, y_ref))
            except Exception as e:
                logger.warning(f"Error evaluating polynomial: {e}")
                continue

            if config.CHECK_FINITE_VALUES and not np.isfinite(x_val):
                logger.warning(f"Non-finite x value: {x_val}")
                continue

            if (x_val < -config.X_VALUE_RANGE_MULTIPLIER * w or
                x_val >  config.X_VALUE_RANGE_MULTIPLIER * w):
                logger.debug(f"x value out of range: {x_val}")
                continue

            xs.append(x_val)

        if len(xs) == 0:
            logger.debug("No valid curves found")
            return None, None, None

        xs = np.array(xs, dtype=np.float32)

        camera_center_x = w / 2.0

        if len(xs) >= 2:
            xs_sorted = np.sort(xs)
            left_x = xs_sorted[0]
            right_x = xs_sorted[-1]
            lane_center_x = (left_x + right_x) / 2.0
        else:
            lane_center_x = xs[0]

        lane_center_x = max(0.0, min(float(w - 1), float(lane_center_x)))

        error_px = lane_center_x - camera_center_x

        logger.debug(f"Lane center error: {error_px:.1f}px")
        return float(error_px), float(lane_center_x), float(camera_center_x)
    
    except Exception as e:
        logger.error(f"Error in get_lane_center_error: {e}")
        return None, None, None


def get_lane_heading_error(poly_list, frame_shape, y_ref_ratio=None):
    """
    poly_list : [ [a3, a2, a1, a0], ... ]
    frame_shape: frame.shape
    y_ref_ratio: get_lane_center_error와 동일한 y 기준 사용

    반환:
        heading_error_rad : 차선 평균 기울기의 rad 단위 heading error
                            > 0  : 차선이 오른쪽으로 기울어짐
                            < 0  : 차선이 왼쪽으로 기울어짐
    """
    try:
        if y_ref_ratio is None:
            y_ref_ratio = config.LANE_Y_REF_RATIO
        
        h, w = frame_shape[:2]
        y_ref = int(h * y_ref_ratio)
        y_ref = max(0, min(h - 1, y_ref))

        slopes = []
        for coeffs in poly_list:
            try:
                if len(coeffs) != 4:
                    logger.warning(f"Invalid coefficient length: {len(coeffs)}")
                    continue
                
                a3, a2, a1, a0 = coeffs
                # dx/dy = 3*a3*y^2 + 2*a2*y + a1
                dx_dy = 3 * a3 * (y_ref ** 2) + 2 * a2 * y_ref + a1

                if config.CHECK_FINITE_VALUES and not np.isfinite(dx_dy):
                    logger.warning(f"Non-finite dx_dy: {dx_dy}")
                    continue
                
                slopes.append(dx_dy)
            except Exception as e:
                logger.warning(f"Error calculating slope: {e}")
                continue

        if len(slopes) == 0:
            logger.debug("No valid slopes found")
            return None

        dx_dy_mean = float(np.mean(slopes))

        lane_angle_rad = float(np.arctan(dx_dy_mean))

        # 카메라 기준 "세로가 정면"이라고 가정 → target = 0 rad
        heading_error_rad = lane_angle_rad

        logger.debug(f"Heading error: {np.degrees(heading_error_rad):.1f} deg")
        return heading_error_rad
    
    except Exception as e:
        logger.error(f"Error in get_lane_heading_error: {e}")
        return None


def compute_steering_command(error_px, heading_error_rad,
                             K_lat=None, K_head=None):
    """
    error_px         : get_lane_center_error에서 나온 픽셀 단위 lateral error
    heading_error_rad: get_lane_heading_error에서 나온 rad 단위 heading error
    K_lat            : lateral error gain (픽셀 → 조향) - None이면 config 값 사용
    K_head           : heading error gain (rad → 조향) - None이면 config 값 사용

    반환:
        steering_cmd : -1.0 ~ +1.0 범위의 normalized 조향 명령
                       > 0 : 오른쪽 조향
                       < 0 : 왼쪽 조향
    """
    try:
        if K_lat is None:
            K_lat = config.K_LATERAL
        if K_head is None:
            K_head = config.K_HEADING
        
        if error_px is None or heading_error_rad is None:
            logger.warning("error_px or heading_error_rad is None")
            return 0.0

        steering = K_lat * error_px + K_head * heading_error_rad

        steering = max(config.STEERING_CMD_MIN,
                       min(config.STEERING_CMD_MAX, steering))

        logger.debug(f"Steering command: {steering:.3f}")
        return steering
    
    except Exception as e:
        logger.error(f"Error in compute_steering_command: {e}")
        return 0.0


def draw_lane_center_debug(frame, poly_list, y_ref_ratio=None):
    """
    frame     : BGR 프레임 (원본)
    poly_list : fit_red_curves(red_mask) 결과
    y_ref_ratio : 기준 y 비율 (0~1) - None이면 config 값 사용
    반환:
        vis : 디버깅 정보가 그려진 BGR 이미지
              - 노란 세로선 : 카메라 중심
              - 하늘색 세로선 : 차선 중심
              - 기준 y 위치에 작은 점들
    """
    try:
        if frame is None or frame.size == 0:
            logger.error("Invalid frame: frame is None or empty")
            return frame
        
        if y_ref_ratio is None:
            y_ref_ratio = config.LANE_Y_REF_RATIO
        
        vis = frame.copy()
        h, w, _ = vis.shape

        error_px, lane_center_x, camera_center_x = get_lane_center_error(
            poly_list, frame.shape, y_ref_ratio
        )

        cam_x = int(round(camera_center_x)) if camera_center_x is not None else w // 2
        cv2.line(vis, (cam_x, 0), (cam_x, h - 1), (0, 255, 255), 1)

        if lane_center_x is not None:
            lane_x = int(round(lane_center_x))
            y_ref = int(h * y_ref_ratio)
            y_ref = max(0, min(h - 1, y_ref))

            cv2.line(vis, (lane_x, 0), (lane_x, h - 1), (255, 255, 0), 1)

            cv2.circle(vis, (lane_x, y_ref), 5, (255, 255, 0), -1)
            cv2.circle(vis, (cam_x, y_ref), 5, (0, 255, 255), -1)

            if error_px is not None:
                text = f"offset: {error_px:.1f} px"
                cv2.putText(
                    vis,
                    text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

        return vis
    
    except Exception as e:
        logger.error(f"Error in draw_lane_center_debug: {e}")
        return frame


def main(stop_event=None, args=None):
    """
    메인 루프: 카메라 입력 → 차선 감지 → 조향 명령 계산 → 시각화
    
    Args:
        stop_event: 스레드 종료 이벤트 (None이면 무한 실행)
        args: 추가 인자 (미사용)
    """
    picam2 = None
    
    try:
        # === 카메라 초기화 ===
        picam2 = Picamera2()
        config_dict = picam2.create_preview_configuration(
            main={
                "size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT),
                # ★ 여기서 BGR888로 고정: OpenCV에서 바로 BGR로 사용
                "format": "BGR888"
            }
        )
        picam2.configure(config_dict)
        picam2.start()
        sleep(1)

        frame_count = 0

        while not (stop_event and stop_event.is_set()):
            try:
                # 1) 프레임 캡처
                # Picamera2에서 이미 BGR888로 받아오므로 추가 색 변환 필요 X
                frame = picam2.capture_array()  # BGR

                # 2) 빨간 라인 마스크
                red_mask = get_red_mask(frame)
                
                # 3) 3차 곡선 피팅
                curves = fit_red_curves(red_mask)

                # 4) lateral error (px)
                error_px, lane_center_x, camera_center_x = get_lane_center_error(
                    curves, frame.shape, y_ref_ratio=config.LANE_Y_REF_RATIO
                )

                # 5) heading error (rad)
                heading_error_rad = get_lane_heading_error(
                    curves, frame.shape, y_ref_ratio=config.LANE_Y_REF_RATIO
                )

                # 6) 조향 명령 계산
                steering_cmd = compute_steering_command(
                    error_px,
                    heading_error_rad,
                    K_lat=config.K_LATERAL,
                    K_head=config.K_HEADING
                )
                globalVar.LKSangle = steering_cmd

                # 7) 디버깅 시각화
                if config.DEBUG_VISUALIZATION_ENABLED:
                    lane_center_vis = draw_lane_center_debug(
                        frame, curves, y_ref_ratio=config.LANE_Y_REF_RATIO
                    )

                    # heading 디버그 출력
                    if heading_error_rad is not None:
                        text2 = f"heading: {np.degrees(heading_error_rad):.1f} deg"
                        cv2.putText(
                            lane_center_vis,
                            text2,
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (255, 0, 0),
                            2,
                        )

                    cv2.imshow("Lane Center Debug", lane_center_vis)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                frame_count += 1
            
            except Exception as e:
                logger.error(f"Error in main loop: {e}")
                continue

    except Exception as e:
        logger.error(f"Fatal error in main: {e}")
    
    finally:
        try:
            if picam2 is not None:
                picam2.stop()
        except:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
