import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep

# ---------------------------------------------------
# 1. 라인 마스크 생성
# ---------------------------------------------------
def get_red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    red_mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    return red_mask


# ---------------------------------------------------
# 2. 라인 직선 피팅
# ---------------------------------------------------
def fit_red_lines(red_mask):
    contours, _ = cv2.findContours(
        red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )

    if len(contours) < 1:
        return []
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    contours = contours[:2]

    line_params_list = []

    for cnt in contours:
        vx, vy, x0, y0 = cv2.fitLine(
            cnt, cv2.DIST_L2, 0, 0.01, 0.01
        )
        a = float(vy)
        b = float(-vx)
        c = float(vx * y0 - vy * x0)
        line_params_list.append((a, b, c))

    return line_params_list


# ---------------------------------------------------
# 3. 직선 그리기
# ---------------------------------------------------
def draw_lines(frame, line_list):
    h, w, _ = frame.shape

    for (a, b, c) in line_list:
        if abs(b) < 1e-6:
            continue
        x0, x1 = 0, w
        y0 = int(-(a * x0 + c) / b)
        y1 = int(-(a * x1 + c) / b)

        cv2.line(frame, (x0, y0), (x1, y1), (0, 0, 255), 2)

    return frame


# ---------------------------------------------------
# 4. 바운딩 박스가 도로 안(in road)인지 판별
#    - line_params_list : fit_red_lines()의 결과
#    - bbox            : (x1, y1, x2, y2)
#    - frame_shape     : frame.shape (도로 기준 y좌표 계산용)
#    반환:
#       1 : 도로 안(두 빨간 라인 사이)
#       0 : 도로 밖(라인 사이가 아니거나, 라인 검출 실패)
# ---------------------------------------------------
def getObjInRoad(bbox, line_params_list, frame_shape):
    # 기본값: 도로 밖이라고 가정
    isObjInRoad = 0

    # 빨간 라인이 2개 미만이면 판별 불가 → 0 리턴
    if len(line_params_list) < 2:
        return isObjInRoad

    h, w = frame_shape[:2]
    x1, y1, x2, y2 = bbox

    # 바운딩 박스 아래 중앙점
    cx = (x1 + x2) / 2.0
    cy = float(y2)

    # 각 라인에서 이 y(cy)에서의 x좌표를 계산
    # ax + by + c = 0  ->  x = -(b*y + c) / a  (a != 0 가정)
    xs = []
    for (a, b, c) in line_params_list:
        if abs(a) < 1e-6:
            # a가 0이면 거의 수평인 직선 → 도로 경계로 보기 어려우니 건너뜀
            continue
        x_on_line = -(b * cy + c) / a
        xs.append(x_on_line)

    # 유효한 x가 2개 이상 있어야 "왼쪽/오른쪽 경계"를 만들 수 있음
    if len(xs) < 2:
        return isObjInRoad

    xs.sort()
    left_x = xs[0]
    right_x = xs[-1]

    # 바운딩 박스 중앙이 두 라인 사이에 있으면 도로 안
    if left_x <= cx <= right_x:
        isObjInRoad = 1
    else:
        isObjInRoad = 0

    return isObjInRoad

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
    # 컨투어 추출
    contours, _ = cv2.findContours(
        red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )

    if len(contours) < 1:
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
        if len(xs) < 4:
            continue

        # x = f(y) 형태로 3차 피팅
        coeffs = np.polyfit(ys, xs, 3)  # [a3, a2, a1, a0]
        poly_list.append(coeffs)

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

    cv2.imshow("Red Curve Fit Debug", debug_vis)
    return poly_list

def get_lane_center_error(poly_list, frame_shape, y_ref_ratio=0.9):
    """
    poly_list: fit_red_curves(red_mask)가 반환한 3차 다항식 계수 리스트
               각 원소는 [a3, a2, a1, a0]
               x = a3*y^3 + a2*y^2 + a1*y + a0
    frame_shape: frame.shape  (h, w, c)
    y_ref_ratio: 화면 세로에서 어느 높이에서 기준을 잡을지 (0~1 비율)
                 0.9 -> 화면 아래쪽 90% 지점 근처 (바닥 가까운 지점)

    반환:
        error_px       : lane_center_x - camera_center_x (픽셀 단위 오차)
        lane_center_x  : 기준 y에서 차선 중심 x 좌표
        camera_center_x: 화면 중심 x 좌표 (w/2)

    의미:
        error_px > 0 : 차선 중심이 카메라 기준 오른쪽에 있음
        error_px < 0 : 차선 중심이 카메라 기준 왼쪽에 있음
    """
    h, w = frame_shape[:2]

    # 기준 y (화면 아래쪽 근처)
    y_ref = int(h * y_ref_ratio)
    y_ref = max(0, min(h - 1, y_ref))  # 안전하게 클램프

    xs = []
    for coeffs in poly_list:
        # coeffs: [a3, a2, a1, a0]
        x_val = np.polyval(coeffs, y_ref)
        xs.append(x_val)

    if len(xs) == 0:
        # 곡선이 하나도 없으면 오차 계산 불가
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

    error_px = lane_center_x - camera_center_x

    return float(error_px), float(lane_center_x), float(camera_center_x)


def draw_lane_center_debug(frame, poly_list, y_ref_ratio=0.9):
    """
    frame     : BGR 프레임 (원본)
    poly_list : fit_red_curves(red_mask) 결과
    y_ref_ratio : 기준 y 비율 (0~1)
    반환:
        vis : 디버깅 정보가 그려진 BGR 이미지
              - 노란 세로선 : 카메라 중심
              - 하늘색 세로선 : 차선 중심
              - 기준 y 위치에 작은 점들
    """
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



def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 360), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    sleep(1)

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 1) 빨간 라인 마스크
        red_mask = get_red_mask(frame)

        # 2) 빨간 라인 최대 2개 찾기
        lines = fit_red_lines(red_mask)
        
        
        curves = fit_red_curves(red_mask)

        error_px, lane_center_x, camera_center_x = get_lane_center_error(
            curves, frame.shape, y_ref_ratio=0.9)

        # 3) 디버깅 시각화
        lane_center_vis = draw_lane_center_debug(frame, curves, y_ref_ratio=0.9)
        cv2.imshow("Lane Center Debug", lane_center_vis)
        
        # 3) 도로 위에 임의 bbox 예시 (나중에 YOLO bbox로 대체)
        h, w, _ = frame.shape
        bbox = (w // 2 - 40, h // 2, w // 2 + 40, h // 2 + 80)
        x1, y1, x2, y2 = bbox

        # 4) 도로 안/밖 판별
        in_road = getObjInRoad(bbox, lines, frame.shape)

        # 5) 시각화
        vis = draw_lines(frame.copy(), lines)

        if in_road == 1:
            color = (0, 255, 0)
            text = "IN ROAD"
        else:
            color = (0, 0, 255)
            text = "OUT OF ROAD"

        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        cv2.putText(vis, text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        cv2.imshow("Red Mask", red_mask)
        cv2.imshow("Road In/Out Demo", vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
