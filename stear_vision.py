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
    
    Args:
        frame: BGR 프레임
        
    Returns:
        red_mask: 이진 마스크 (빨간색 영역 = 255)
        
    Raises:
        ValueError: frame이 None이거나 유효하지 않은 경우
    """
    try:
        if frame is None or frame.size == 0:
            raise ValueError("Invalid frame: frame is None or empty")
        
        # RGB → HSV 변환 (프레임이 RGB 포맷이므로)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        lower_red1 = np.array(config.RED_HSV_LOWER1)
        upper_red1 = np.array(config.RED_HSV_UPPER1)
        lower_red2 = np.array(config.RED_HSV_LOWER2)
        upper_red2 = np.array(config.RED_HSV_UPPER2)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=config.MORPH_CLOSE_ITERATIONS)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=config.MORPH_OPEN_ITERATIONS)

        return red_mask
    
    except Exception as e:
        logger.error(f"Error in get_red_mask: {e}")
        raise



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
        
        # 컨투어 추출
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
        # 디버깅용 시각화 이미지 (회색 -> BGR)
        debug_vis = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)

        poly_list = []

        for cnt in contours:
            # cnt: (N,1,2) -> (N,2)
            pts = cnt.reshape(-1, 2)
            xs = pts[:, 0].astype(np.float32)
            ys = pts[:, 1].astype(np.float32)

            # 3차 다항식 피팅에는 최소 4개 점 필요
            if len(xs) < config.MIN_CONTOUR_POINTS:
                logger.debug(f"Contour has less than {config.MIN_CONTOUR_POINTS} points: {len(xs)}")
                continue

            try:
                # x = f(y) 형태로 3차 피팅
                coeffs = np.polyfit(ys, xs, config.POLY_FIT_DEGREE)  # [a3, a2, a1, a0]
                poly_list.append(coeffs)

                if config.DEBUG_VISUALIZATION_ENABLED:
                    # 1) 원래 컨투어 점 시각화 (초록 점)
                    for x, y in zip(xs.astype(int), ys.astype(int)):
                        if 0 <= x < w and 0 <= y < h:
                            cv2.circle(debug_vis, (x, y), 1, (0, 255, 0), -1)

                    # 2) 피팅된 3차 곡선 시각화 (파란 라인)
                    y_min = int(np.clip(ys.min(), 0, h - 1))
                    y_max = int(np.clip(ys.max(), 0, h - 1))
                    if y_max <= y_min:
                        continue

                    # 해당 컨투어 y 구간에서 샘플링
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
                            prev_pt = None  # 화면 밖으로 나가면 끊기
            
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
               x = a3*y^3 + a2*y^2 + a1*y + a0
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

        # 기준 y (화면 아래쪽 근처)
        y_ref = int(h * y_ref_ratio)
        y_ref = max(0, min(h - 1, y_ref))  # 안전하게 클램프

        xs = []
        for coeffs in poly_list:
            try:
                if len(coeffs) != 4:
                    logger.warning(f"Invalid coefficient length: {len(coeffs)}")
                    continue
                
                # coeffs: [a3, a2, a1, a0]
                x_val = float(np.polyval(coeffs, y_ref))
            except Exception as e:
                # 다항식 평가 실패(형식 이상 등)
                logger.warning(f"Error evaluating polynomial: {e}")
                continue

            # 필터: 유한값이어야 하고, 합리적인 범위 내에 있어야 함
            if config.CHECK_FINITE_VALUES and not np.isfinite(x_val):
                logger.warning(f"Non-finite x value: {x_val}")
                continue

            # 프레임 폭을 기준으로 너무 벗어난 값은 이상치로 간주
            # (예: 카메라 중심에서 10배 이상 벗어나면 무시)
            if x_val < -config.X_VALUE_RANGE_MULTIPLIER * w or x_val > config.X_VALUE_RANGE_MULTIPLIER * w:
                logger.debug(f"x value out of range: {x_val}")
                continue

            xs.append(x_val)

        if len(xs) == 0:
            # 곡선이 하나도 없으면 오차 계산 불가
            logger.debug("No valid curves found")
            return None, None, None

        xs = np.array(xs, dtype=np.float32)

        # 카메라 중심 (이미지 중심)
        camera_center_x = w / 2.0

        if len(xs) >= 2:
            # 두 개 이상이면 왼쪽/오른쪽 차선이라고 보고 중앙값 사용
            xs_sorted = np.sort(xs)
            left_x = xs_sorted[0]
            right_x = xs_sorted[-1]
            lane_center_x = (left_x + right_x) / 2.0
        else:
            # 하나만 잡히면 그걸 차선 중심으로 간주
            lane_center_x = xs[0]

        # 차선 중심값을 프레임 내부로 클램프(안정성)
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

        # 여러 개 있으면 평균
        dx_dy_mean = float(np.mean(slopes))

        # rad 단위 기울기
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

        # 여기서는 "lane_center가 오른쪽이면 +error_px"이므로
        # steering_cmd도 오른쪽 +가 되도록 부호를 맞춰줌
        steering = K_lat * error_px + K_head * heading_error_rad

        # 안정성을 위해 saturation
        steering = max(config.STEERING_CMD_MIN, min(config.STEERING_CMD_MAX, steering))

        logger.debug(f"Steering command: {steering:.3f}")
        return steering
    
    except Exception as e:
        logger.error(f"Error in compute_steering_command: {e}")
        return 0.0


def draw_lane_center_debug(frame, poly_list, y_ref_ratio=None):
    """
    frame     : BGR 프레임 (cv2.imshow 사용을 위해)
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

        # 카메라 중심선 그리기 (노란색)
        cam_x = int(round(camera_center_x)) if camera_center_x is not None else w // 2
        cv2.line(vis, (cam_x, 0), (cam_x, h - 1), (0, 255, 255), 1)

        if lane_center_x is not None:
            lane_x = int(round(lane_center_x))
            y_ref = int(h * y_ref_ratio)
            y_ref = max(0, min(h - 1, y_ref))

            # 차선 중심선 그리기 (하늘색)
            cv2.line(vis, (lane_x, 0), (lane_x, h - 1), (255, 255, 0), 1)

            # 기준 y 위치 표시
            cv2.circle(vis, (lane_x, y_ref), 5, (255, 255, 0), -1)
            cv2.circle(vis, (cam_x, y_ref), 5, (0, 255, 255), -1)

            # 텍스트로 오차 표시
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
    retry_count = 0
    
    try:
        # 카메라 초기화
        try:
            picam2 = Picamera2()
            config_dict = picam2.create_preview_configuration(
                main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT), "format": "RGB888"}
            )
            picam2.configure(config_dict)
            picam2.start()
            sleep(1)
            logger.info("Camera initialized successfully")
        except Exception as e:
            logger.error(f"Camera initialization failed: {e}")
            raise

        # 메인 루프
        frame_count = 0
        while not (stop_event and stop_event.is_set()):
            try:
                # 1) 프레임 캡처
                frame = picam2.capture_array()
                if frame is None or frame.size == 0:
                    logger.warning("Invalid frame captured")
                    retry_count += 1
                    if retry_count > config.MAX_RETRIES:
                        logger.error(f"Failed to capture valid frame after {config.MAX_RETRIES} retries")
                        break
                    sleep(config.RETRY_DELAY)
                    continue
                
                retry_count = 0  # 정상 프레임 시 카운터 리셋
                
                # RGB 포맷 유지 (Picamera2에서 RGB888로 반환)
                # frame은 이미 RGB 포맷

                # 2) 빨간 라인 마스크
                try:
                    red_mask = get_red_mask(frame)
                except Exception as e:
                    logger.warning(f"Error in get_red_mask: {e}")
                    red_mask = None
                
                # 3) 3차 곡선 피팅
                try:
                    curves = fit_red_curves(red_mask) if red_mask is not None else []
                except Exception as e:
                    logger.warning(f"Error in fit_red_curves: {e}")
                    curves = []

                # 4) lateral error (px 단위)
                try:
                    error_px, lane_center_x, camera_center_x = get_lane_center_error(
                        curves, frame.shape, y_ref_ratio=config.LANE_Y_REF_RATIO
                    )
                except Exception as e:
                    logger.warning(f"Error in get_lane_center_error: {e}")
                    error_px, lane_center_x, camera_center_x = None, None, None

                # 5) heading error (rad 단위)
                try:
                    heading_error_rad = get_lane_heading_error(
                        curves, frame.shape, y_ref_ratio=config.LANE_Y_REF_RATIO
                    )
                except Exception as e:
                    logger.warning(f"Error in get_lane_heading_error: {e}")
                    heading_error_rad = None

                # 6) 조향 명령 계산
                try:
                    steering_cmd = compute_steering_command(
                        error_px,
                        heading_error_rad,
                        K_lat=config.K_LATERAL,
                        K_head=config.K_HEADING
                    )
                    globalVar.LKSangle = steering_cmd
                except Exception as e:
                    logger.warning(f"Error in compute_steering_command: {e}")
                    steering_cmd = 0.0

                # TODO: steering_cmd를 실제 모터/서보 제어로 변환
                # 예시)
                # pwm_val = int(config.PWM_NEUTRAL + steering_cmd * config.PWM_STEERING_RANGE)
                # pwm_val = max(config.PWM_MIN, min(config.PWM_MAX, pwm_val))
                # send_pwm_to_servo(pwm_val)

                # 7) 디버깅 시각화
                if config.DEBUG_VISUALIZATION_ENABLED:
                    try:
                        # 시각화를 위해 RGB → BGR 변환 (cv2.imshow()가 BGR을 기대)
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        lane_center_vis = draw_lane_center_debug(
                            frame_bgr, curves, y_ref_ratio=config.LANE_Y_REF_RATIO
                        )

                        # heading error 디버깅 텍스트
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
                        
                        # steering command 표시
                        text3 = f"steering: {steering_cmd:.3f}"
                        cv2.putText(
                            lane_center_vis,
                            text3,
                            (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (200, 100, 0),
                            2,
                        )

                        cv2.imshow("Lane Center Debug", lane_center_vis)
                    except Exception as e:
                        logger.warning(f"Error in visualization: {e}")

                # 8) 종료 키 처리
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    logger.info("Quit signal received")
                    break

                frame_count += 1
                if frame_count % 100 == 0:
                    logger.info(f"Processed {frame_count} frames")

            except KeyboardInterrupt:
                logger.info("KeyboardInterrupt received")
                break
            except Exception as e:
                logger.error(f"Error in main loop: {e}")
                retry_count += 1
                if retry_count > config.MAX_RETRIES:
                    logger.error(f"Too many errors, exiting")
                    break
                sleep(config.RETRY_DELAY)
                continue

    except Exception as e:
        logger.error(f"Fatal error in main: {e}")
    
    finally:
        # 정리
        try:
            if picam2 is not None:
                picam2.stop()
                logger.info("Camera stopped")
        except Exception as e:
            logger.error(f"Error stopping camera: {e}")
        
        try:
            cv2.destroyAllWindows()
            logger.info("All windows closed")
        except Exception as e:
            logger.error(f"Error closing windows: {e}")
        
        logger.info("Program terminated")



if __name__ == "__main__":
    main()
