import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep

from motorControl import lonControl, latControl  # 내가 만든 모터 제어 함수들


# ---------------------------------------------------
# 1. 차선 엣지(Canny) 추출
# ---------------------------------------------------
def get_lane_edges(frame):
    """
    입력: BGR 프레임 (카메라 원본)
    출력: edges (Canny 결과, 단일 채널 이미지)
    """
    # 그레이스케일 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 노이즈 제거를 위한 가우시안 블러
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Canny 엣지 검출
    edges = cv2.Canny(blurred, 50, 150)

    return edges


# ---------------------------------------------------
# 2. 기준 y(y_ref)에서 차선 엣지 기반 차선 중앙 x 위치 찾기
# ---------------------------------------------------
def find_lane_center_from_edges(edges, y_ref):
    """
    edges 이미지에서 y_ref 라인의 픽셀을 기준으로
    - 왼쪽 차선 / 오른쪽 차선의 엣지 위치를 찾고
    - 그 중간 x값(heading_x)을 계산한다.

    반환:
        heading_x (int) 또는 None (차선 검출 실패 시)
        left_x, right_x (디버깅용, 없으면 None)
    """
    h, w = edges.shape

    # y_ref가 이미지 범위를 벗어나지 않도록 보정
    y_ref = max(0, min(h - 1, y_ref))

    # 해당 y라인의 픽셀 값 (1차원 배열)
    row = edges[y_ref, :]

    # 화면 중앙 기준으로 왼쪽 / 오른쪽 구간 나누기
    center = w // 2

    # ---------- 왼쪽 차선 탐색 ----------
    # center 기준 왼쪽 부분
    left_part = row[:center]
    # 엣지가 있는 위치(픽셀 값 > 0)
    left_indices = np.where(left_part > 0)[0]
    left_x = None
    if len(left_indices) > 0:
        # center 쪽에 가장 가까운 엣지 픽셀 사용
        left_x = left_indices[-1]

    # ---------- 오른쪽 차선 탐색 ----------
    # center 기준 오른쪽 부분
    right_part = row[center:]
    right_indices = np.where(right_part > 0)[0]
    right_x = None
    if len(right_indices) > 0:
        # center 쪽에 가장 가까운 엣지 픽셀 사용
        right_x = center + right_indices[0]

    # 양쪽 차선 중 하나라도 못 찾으면 실패 처리
    if left_x is None or right_x is None:
        return None, left_x, right_x

    # 왼쪽 차선과 오른쪽 차선의 중간이 차량이 따라가야 할 x 위치
    heading_x = (left_x + right_x) // 2

    return heading_x, left_x, right_x


# ---------------------------------------------------
# 3. 차선 중앙 기준 조향 각도 계산 + 시각화
# ---------------------------------------------------
def compute_steering_angle(frame, max_steering_deg=40):
    """
    - Canny Edge로 차선 엣지 검출
    - 화면 아래쪽(예: 높이의 70%) 위치(y_ref)에서
      왼쪽/오른쪽 차선을 찾고 그 중간 지점을 heading_x로 사용
    - 화면 중앙(center_x)과 heading_x의 차이를 기반으로
      정규화 후 조향 각도(steering_angle)를 계산

    반환:
        steering_angle (deg, float)
        vis (BGR 시각화용 이미지)
    """
    h, w, _ = frame.shape

    # 1) 엣지 이미지 생성
    edges = get_lane_edges(frame)

    # 2) 화면 높이의 70% 지점을 기준 y로 사용
    y_ref = int(h * 0.7)
    heading_x, left_x, right_x = find_lane_center_from_edges(edges, y_ref)

    # 시각화용 이미지
    vis = frame.copy()
    center_x = w // 2

    # 기준 y 라인 그리기
    cv2.line(vis, (0, y_ref), (w, y_ref), (255, 255, 0), 1)

    steering_angle = 0.0

    # 화면 중앙 표시
    cv2.circle(vis, (center_x, y_ref), 6, (0, 0, 0), -1)

    # 화면 중앙에서 아래로 떨어지는 선(참고용)
    cv2.line(vis, (center_x, y_ref), (center_x, h), (255, 0, 0), 2)

    if heading_x is not None:
        # 왼쪽 차선/오른쪽 차선 지점 표시
        if left_x is not None:
            cv2.circle(vis, (left_x, y_ref), 5, (0, 0, 255), -1)
        if right_x is not None:
            cv2.circle(vis, (right_x, y_ref), 5, (255, 0, 0), -1)

        # 차선 중앙(heading_x) 표시
        cv2.circle(vis, (heading_x, y_ref), 6, (0, 255, 255), -1)

        # ---------- 디버깅용 거리 표시 ----------
        # 왼쪽 차선까지의 거리
        if left_x is not None:
            left_dist = center_x - left_x
            cv2.line(vis, (left_x, y_ref), (center_x, y_ref), (0, 0, 255), 2)
            cv2.putText(
                vis,
                f"Left Dist: {left_dist}px",
                (left_x + 5, y_ref - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        # 오른쪽 차선까지의 거리
        if right_x is not None:
            right_dist = right_x - center_x
            cv2.line(vis, (center_x, y_ref), (right_x, y_ref), (255, 0, 0), 2)
            cv2.putText(
                vis,
                f"Right Dist: {right_dist}px",
                (right_x - 160, y_ref - 10),  # 글자 위치 조절
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
            )

        # ---------- 조향 각도 계산 ----------
        # 화면 중앙 기준으로 차선 중앙이 얼마나 어긋났는지 (픽셀 단위)
        # heading_deviation > 0 : 차선이 화면 왼쪽에 있음 (좌측으로 치우침)
        heading_deviation = center_x - heading_x

        # 화면 폭의 1/4을 기준으로 정규화 (-1 ~ 1)
        normalized_deviation = heading_deviation / (w // 4)
        normalized_deviation = np.clip(normalized_deviation, -1.0, 1.0)

        # 최댓값을 max_steering_deg로 하는 비례 조향 각도
        steering_angle = normalized_deviation * max_steering_deg

    else:
        # 차선을 못 찾으면 조향각 0으로 (직진 유지)
        steering_angle = 0.0

    # 화면 왼쪽 상단에 조향각 표시
    cv2.putText(
        vis,
        f"Steer: {steering_angle:.1f} deg",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )

    return steering_angle, vis


# ---------------------------------------------------
# 4. 수동 속도 제어 + 차선 기반 조향 제어 루프
# ---------------------------------------------------
def drive_lane_manual_speed():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 360), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    sleep(1)

    # -100 ~ 100 범위에서 사용 (lonControl 입력)
    current_speed = 0

    print("[주행 시작]")
    print("  w: 속도 +10")
    print("  s: 속도 -10")
    print("  space: 정지 (speed=0)")
    print("  q: 종료")
    print("조향은 Canny 기반 차선 인식을 이용해서 자동으로 제어합니다.\n")

    try:
        while True:
            # 카메라 프레임 캡처
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # 1) 차선 기반 조향각 계산
            steering_angle, vis = compute_steering_angle(frame)

            # 2) 모터 제어
            lonControl(current_speed)   # 종방향(속도) 제어: 사용자가 키보드로 조절
            latControl(steering_angle)  # 횡방향(조향) 제어: 차선 기준 자동 조향

            # 현재 속도 정보 화면에 출력
            cv2.putText(
                vis,
                f"Speed: {current_speed}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            cv2.imshow("Lane Edge & Steering", vis)

            # 3) 키보드 입력 처리
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('w'):
                current_speed += 10
                if current_speed > 100:
                    current_speed = 100
                print(f"[Speed Up] {current_speed}")
            elif key == ord('s'):
                current_speed -= 10
                if current_speed < -100:
                    current_speed = -100
                print(f"[Speed Down] {current_speed}")
            elif key == ord(' '):
                current_speed = 0
                print("[STOP] speed = 0")

    finally:
        # 자원 정리
        picam2.stop()
        cv2.destroyAllWindows()
        lonControl(0)
        latControl(0)
        print("Stopped car.")


if __name__ == "__main__":
    print("Start Canny-based lane following (manual speed)...")
    drive_lane_manual_speed()
