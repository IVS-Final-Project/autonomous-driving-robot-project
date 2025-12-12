# =============================================================================
# 로봇 제어 설정 파일
# =============================================================================

# ===== [카메라 및 비전 설정] =====
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360
CAMERA_FORMAT = "BGR888"

# 빨간 라인 감지 HSV 범위
RED_HSV_LOWER1 = (0, 100, 100)
RED_HSV_UPPER1 = (10, 255, 255)
RED_HSV_LOWER2 = (170, 100, 100)
RED_HSV_UPPER2 = (180, 255, 255)

# 형태학 연산 커널 설정
MORPH_KERNEL_SIZE = 3
MORPH_CLOSE_ITERATIONS = 2
MORPH_OPEN_ITERATIONS = 1

# ===== [차선 인식 설정] =====
# 다항식 피팅 차수 (3차 곡선)
POLY_FIT_DEGREE = 3

# 기준 y 위치 비율 (화면 아래쪽 근처, 0~1)
LANE_Y_REF_RATIO = 0.9

# 컨투어 최소 포인트 개수
MIN_CONTOUR_POINTS = 4

# ===== [조향 제어 설정] =====
# Lateral error 게인 (픽셀 → 조향 명령)
K_LATERAL = 0.003

# Heading error 게인 (rad → 조향 명령)
K_HEADING = 0.8

# 조향 명령 범위
STEERING_CMD_MIN = -1.0
STEERING_CMD_MAX = 1.0

# ===== [속도 제어 설정] =====
ZONE_IDS = {
    'IDLE': 0,
    'CHILD': 1,
    'HIGHACCIDENT': 2,
    'SPEEDBUMP': 3
}

VELOCITY_IDLE = 30
VELOCITY_CHILD_ZONE = 13
VELOCITY_HIGH_ACCIDENT_ZONE = 20
VELOCITY_SPEED_BUMP = 13
VELOCITY_OBJ_IN_ROAD = 10

# ===== [ArUco 마커 설정] =====
ARUCO_DICT_TYPE = "DICT_4X4_250"

ZONE_MAP = {
    0: "Normal",
    1: "Child",
    2: "Accident",
    3: "Bump"
}

# ===== [Hailo 객체 감지 설정] =====
OBSTACLE_CLASSES = ['people', 'car']
COLLISION_AREA_THRESHOLD = 0.15

# ===== [모터 제어 설정] =====
# PWM 기본값 (중립 상태)
PWM_NEUTRAL = 128

# PWM 범위
PWM_MIN = 0
PWM_MAX = 255

# 조향 제어 범위 (neutral 기준 ±)
PWM_STEERING_RANGE = 60

# ===== [로깅 설정] =====
# 디버그 윈도우 표시 여부
DEBUG_VISUALIZATION_ENABLED = True

# 콘솔 로깅 레벨 ('DEBUG', 'INFO', 'WARNING', 'ERROR')
LOG_LEVEL = 'INFO'

# ===== [GStreamer 설정] =====
HAILO_HEF_PATH = "/usr/local/hailo/resources/models/hailo8/yolov8m.hef"
HAILO_POST_PROCESS_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"

# 카메라 프레임 기본 포맷
CAMERA_INPUT_FORMAT = "RGB"
CAMERA_INPUT_WIDTH = 640
CAMERA_INPUT_HEIGHT = 480

# 추론 입력 해상도
INFERENCE_WIDTH = 640
INFERENCE_HEIGHT = 640

# 파이프라인 큐 설정
QUEUE_MAX_BUFFERS = 3

# ===== [루프 주기 설정 (초)] =====
MAIN_LOOP_INTERVAL = 0.1
ZONE_CHECK_INTERVAL = 1.0
GSTREAMER_ITERATION_INTERVAL = 0.01

# ===== [오류 재시도 설정] =====
MAX_RETRIES = 3
RETRY_DELAY = 0.5  # 초

# ===== [안정성 설정] =====
# 다항식 평가 결과가 유한값인지 확인 여부
CHECK_FINITE_VALUES = True

# x 값의 합리적인 범위 (프레임 폭의 배수)
X_VALUE_RANGE_MULTIPLIER = 10
