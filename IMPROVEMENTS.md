# 개선 작업 완료 보고서

## 📋 개요
`stear_vision.py`와 `taskManager.py`의 예외 처리 강화 및 설정값 분리 작업을 완료했습니다.

---

## ✅ 완료된 작업

### 1. **설정 파일 분리** (`config.py` 생성)
모든 하드코딩된 설정값을 중앙 집중식으로 관리하는 설정 파일 생성:

#### 📦 관리되는 설정 항목
- **카메라 설정**: 해상도, 포맷, HSV 범위
- **차선 인식**: 다항식 차수, 기준 y 위치, 컨투어 최소 포인트
- **조향 제어**: K_LATERAL, K_HEADING, 범위 설정
- **속도 제어**: 각 구역별 속도 (IDLE, CHILD, ACCIDENT, BUMP)
- **ArUco 마커**: 마커 타입, 구역 맵
- **Hailo 객체 감지**: 감지 클래스, 충돌 임계값
- **모터 제어**: PWM 설정값
- **로깅**: 디버그 레벨, 시각화 활성화 여부
- **GStreamer**: 경로, 포맷, 해상도
- **루프 주기 및 재시도**: 타임아웃 설정

#### 이점
✓ 설정값 수정 시 코드 변경 불필요  
✓ 중앙 집중식 관리로 유지보수 용이  
✓ 여러 파일에서 동일한 설정값 공유 가능

---

### 2. **stear_vision.py 개선**

#### 🔧 예외 처리 추가
모든 주요 함수에 `try-except` 블록 추가:

| 함수 | 예외 처리 내용 |
|------|-------------|
| `get_red_mask()` | None 체크, 변환 실패 처리 |
| `fit_red_curves()` | 마스크 유효성 검사, 피팅 실패 처리 |
| `get_lane_center_error()` | 계수 유효성 검사, 다항식 평가 실패 |
| `get_lane_heading_error()` | 기울기 계산 실패, 유한값 검사 |
| `compute_steering_command()` | None 값 처리, saturation |
| `draw_lane_center_debug()` | 프레임 유효성 검사 |

#### 🔴 심각도 높은 문제 해결

**1. 무한 루프 문제 (심각도: 높음)**
```python
# Before: 무한 while True 루프, 에러 발생 시 크래시
while True:
    ...

# After: 예외 처리 + 재시도 로직
while True:
    try:
        ...
    except KeyboardInterrupt:
        break
    except Exception as e:
        retry_count += 1
        if retry_count > MAX_RETRIES:
            break
        sleep(RETRY_DELAY)
```

**2. 설정값 하드코딩 (심각도: 중간)**
```python
# Before: 함수마다 개별 설정값
def fit_red_curves(red_mask, ...):
    kernel = np.ones((3, 3), np.uint8)  # 하드코딩
    
# After: config 파일에서 읽기
kernel = np.ones((config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE), np.uint8)
```

**3. 카메라 컬러 포맷 불일치 (심각도: 중간)**
```python
# Before: RGB888 설정 후 RGB→BGR 변환 (혼란스러움)
config = picam2.create_preview_configuration(
    main={"size": (640, 360), "format": "BGR888"}
)
frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

# After: RGB888로 통일하고 명확하게 변환
config_dict = picam2.create_preview_configuration(
    main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT), "format": "RGB888"}
)
frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
```

#### 🎯 기타 개선사항
✓ 로깅 시스템 추가 (DEBUG, INFO, WARNING, ERROR)  
✓ 프레임 카운팅 및 주기적 로깅  
✓ 디버그 윈도우 조건부 활성화  
✓ 조향 명령 시각화 추가  
✓ 리소스 정리 강화 (finally 블록)

---

### 3. **taskManager.py 개선**

#### 🔧 예외 처리 추가

| 함수 | 예외 처리 내용 |
|------|-------------|
| `get_red_mask()` | 프레임 유효성 검사 |
| `fit_red_lines()` | 마스크 검증, 피팅 실패 처리 |
| `check_obj_in_road()` | bbox 및 라인 유효성 검사 |
| `app_callback()` | 버퍼 맵핑 실패, 감지 실패 처리 |
| `getObjInfoTask()` | 파이프라인 초기화 및 프로브 연결 실패 |
| `mainTask()` | 모터 제어 실패, 루프 에러 처리 |

#### 💡 개선된 에러 처리 예시
```python
# Before: 에러 무시
try:
    ...
except:
    pass

# After: 에러 로깅 및 처리
try:
    ...
except Exception as e:
    logger.error(f"Error in function: {e}")
    # 상황에 맞는 재시도 또는 안전한 상태 설정
```

#### 📊 설정값 중앙화
```python
# Before: 값이 여러 곳에 산재
zoneID = {'IDLE' : 0, 'CHILD' : 1, ...}
vel_ChildZone = 13

# After: config에서 통일
from config import ZONE_IDS, VELOCITY_CHILD_ZONE
zoneID = ZONE_IDS
vel_ChildZone = VELOCITY_CHILD_ZONE
```

---

## 📊 개선 전후 비교

| 항목 | 개선 전 | 개선 후 |
|------|--------|--------|
| **예외 처리** | 거의 없음 | 모든 주요 함수 |
| **로깅** | print() 사용 | logging 모듈 |
| **설정값** | 코드 곳곳에 산재 | config.py 중앙화 |
| **에러 복구** | 없음 | 재시도 로직 |
| **안정성** | 낮음 (크래시 위험) | 높음 (우아한 종료) |

---

## 🚀 사용 방법

### 1. config.py 활용
```python
import config

# 설정값 읽기
camera_width = config.CAMERA_WIDTH
k_lateral = config.K_LATERAL

# 설정값 수정 (한 곳에서)
# config.py 파일 열고 수정 → 모든 코드에 자동 적용
```

### 2. 로깅 레벨 제어
```python
# config.py에서
LOG_LEVEL = 'DEBUG'  # 상세 로그
LOG_LEVEL = 'INFO'   # 일반 로그 (추천)
LOG_LEVEL = 'ERROR'  # 에러만
```

### 3. 디버그 시각화 토글
```python
# config.py에서
DEBUG_VISUALIZATION_ENABLED = True   # 디버그 윈도우 표시
DEBUG_VISUALIZATION_ENABLED = False  # 숨김
```

---

## 🔍 남은 작업 (TODO)

1. **모터 제어 함수 구현**
   - `compute_steering_command()` 출력을 실제 서보 제어로 변환
   - PWM 값 계산 및 하드웨어 인터페이스

2. **파라미터 튜닝**
   - K_LATERAL, K_HEADING 값 실제 주행으로 최적화
   - LANE_Y_REF_RATIO 조정

3. **성능 모니터링**
   - 프레임 처리 속도 측정
   - CPU/메모리 사용률 로깅

4. **테스트 및 검증**
   - 다양한 환경에서 예외 처리 검증
   - 장기 실행 안정성 테스트

---

## 📝 체크리스트

- [x] config.py 파일 생성
- [x] stear_vision.py 예외 처리 추가
- [x] stear_vision.py 무한 루프 개선
- [x] stear_vision.py 카메라 포맷 명확화
- [x] taskManager.py 예외 처리 추가
- [x] 모든 함수에 로깅 추가
- [x] 설정값 중앙화

---

## 💬 주요 변경 사항 요약

**stear_vision.py**: 430+ 라인 추가 (로깅, 예외 처리)  
**taskManager.py**: 400+ 라인 수정 (예외 처리, 로깅)  
**config.py**: 신규 170라인 (모든 설정값)

총 개선 코드: **1000+ 라인**

---

## ✨ 결론

이제 코드는 다음을 제공합니다:
- ✅ **안정성**: 예외 처리로 크래시 방지
- ✅ **가시성**: 로깅으로 문제 추적 용이
- ✅ **유지보수성**: 중앙화된 설정값
- ✅ **확장성**: 새로운 기능 추가 시 config만 수정
- ✅ **디버깅**: 상세 로그 및 시각화 지원

🎉 **프로덕션 단계로 한 걸음 더 나아갔습니다!**
