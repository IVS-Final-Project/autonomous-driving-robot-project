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
