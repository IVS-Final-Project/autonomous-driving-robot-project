# 🤖 자율주행 로봇 시스템 전체 검토

## 📊 시스템 아키텍처

```
main.py (메인 진입점)
│
├─ 스레드 1: stearVisionThread
│   └─→ stear_vision.py::main()
│       ├─ Picamera2 카메라 초기화
│       ├─ 프레임 입력
│       ├─ 빨간 라인 감지 (get_red_mask)
│       ├─ 3차 곡선 피팅 (fit_red_curves)
│       ├─ 차선 중심 오차 계산 (get_lane_center_error)
│       ├─ 조향 기울기 오차 계산 (get_lane_heading_error)
│       ├─ 조향 명령 계산 (compute_steering_command)
│       └─ globalVar.LKSangle ← 조향값 저장
│
├─ 스레드 2: objInfoTaskThread
│   └─→ taskManager.py::getObjInfoTask()
│       ├─ GStreamer 파이프라인 초기화
│       ├─ Hailo AI 객체 감지 시작
│       ├─ ArUco 마커 인식 (in app_callback)
│       │   └─ globalVar.zoneInfo ← 구역 정보
│       ├─ 객체 감지 (in app_callback)
│       │   └─ globalVar.isObjDetected ← 객체 유무
│       └─ 도로 위치 판별 (in app_callback)
│           └─ globalVar.isObjInRoad ← 도로 위치
│
├─ 스레드 3: mainTaskThread
│   └─→ taskManager.py::mainTask()
│       ├─ 센서 정보 읽기
│       │   ├─ globalVar.zoneInfo
│       │   ├─ globalVar.isObjDetected
│       │   ├─ globalVar.isObjInRoad
│       │   └─ globalVar.userTargetSpeed
│       ├─ 속도 결정 로직
│       │   ├─ IDLE: 사용자 입력 속도 사용
│       │   ├─ CHILD: 13으로 고정
│       │   ├─ HIGHACCIDENT: 20으로 고정
│       │   └─ SPEEDBUMP: 13으로 고정
│       ├─ 장애물 감지 시 정지 (0으로 설정)
│       ├─ globalVar.desiredSpeed ← 최종 속도
│       └─ lonControl(final_speed) ← 모터 제어
│
└─ 메인 스레드: 키보드 입력 처리
    ├─ W/S: 속도 조절
    │   └─ globalVar.userTargetSpeed ±= SPEED_STEP
    ├─ A/D: 조향 조절
    │   └─ globalVar.userTargetAngle ±= ANGLE_STEP
    ├─ 최종 조향값 계산
    │   └─ globalVar.desiredAngle = userTargetAngle + LKSangle ⭐
    ├─ latControl(final_angle) ← 서보 제어
    └─ Q: 프로그램 종료
```

---

## 🔄 데이터 흐름

### 1️⃣ 차선 추적 (Steering)
```
카메라 프레임 (RGB888)
    ↓ stear_vision.py
빨간 라인 감지 (HSV 마스크)
    ↓
3차 곡선 피팅 (컨투어 → 다항식)
    ↓
차선 중심 오차 계산 (픽셀)
차선 기울기 오차 계산 (rad)
    ↓
조향 명령 계산 (K_lat * error_px + K_head * heading_rad)
    ↓
globalVar.LKSangle = steering_cmd
    ↓ (main.py에서 읽음)
최종 조향값 = userTargetAngle + LKSangle
    ↓
latControl(angle) → servo.angle = angle + 2
```

### 2️⃣ 속도 제어 (Speed)
```
taskManager.py::mainTask()
    ↓
구역 정보 확인 (zoneInfo)
    ↓ 
├─ IDLE → userTargetSpeed 사용
├─ CHILD → 13
├─ HIGHACCIDENT → 20
└─ SPEEDBUMP → 13
    ↓
장애물 감지 시 0으로 덮어쓰기
    ↓
globalVar.desiredSpeed = final_speed
    ↓
lonControl(speed) → Motor 제어
```

### 3️⃣ 객체 & 구역 감지 (Detection)
```
Hailo AI (taskManager.py::app_callback)
    ↓
ArUco 마커 인식 → zoneInfo 업데이트
객체 감지 (YOLOv8m) → isObjDetected
도로 위치 판별 → isObjInRoad
```

---

## ⚙️ 주요 설정값 (config.py)

| 설정 | 값 | 용도 |
|------|-----|------|
| `K_LATERAL` | 0.003 | 차선 중심 오차 → 조향 게인 |
| `K_HEADING` | 0.8 | 차선 기울기 오차 → 조향 게인 |
| `LANE_Y_REF_RATIO` | 0.9 | 기준 위치 (화면 아래 90%) |
| `VELOCITY_CHILD_ZONE` | 13 | 어린이 보호 구역 속도 |
| `VELOCITY_HIGH_ACCIDENT_ZONE` | 20 | 사고 위험 구역 속도 |
| `COLLISION_AREA_THRESHOLD` | 0.15 | 충돌 감지 임계값 |

---

## 🔐 GlobalVar 변수 흐름

| 변수 | 설정 위치 | 읽기 위치 | 용도 |
|------|---------|---------|------|
| `zoneInfo` | taskManager (ArUco) | taskManager | 현재 구역 |
| `isObjDetected` | taskManager (Hailo) | taskManager | 객체 감지 유무 |
| `isObjInRoad` | taskManager (Hailo) | taskManager | 도로 위 객체 |
| `userTargetSpeed` | main (키보드) | taskManager | 사용자 목표 속도 |
| `userTargetAngle` | main (키보드) | main | 사용자 목표 조향 |
| `desiredSpeed` | taskManager | main | 최종 속도 명령 |
| `desiredAngle` | main | main | 최종 조향 명령 |
| `LKSangle` | stear_vision | main | 자동 조향값 |

---

## 🚨 잠재적 문제점 및 개선 필요 사항

### 1️⃣ **globalVar 초기화 누락**
**상태**: ⚠️ **심각도: 중간**

```python
# globalVar.py에 누락된 변수
userTargetSpeed = 0      # ❌ 없음
userTargetAngle = 0      # ❌ 없음
```

**해결**: globalVar.py에 다음 추가 필요
```python
userTargetSpeed = 0
userTargetAngle = 0
```

### 2️⃣ **초기값 문제**
**상태**: ⚠️ **심각도: 중간**

main.py 실행 시 `LKSangle`이 초기화되지 않으면:
```python
globalVar.desiredAngle = 0 + None  # TypeError!
```

**해결**: globalVar.py에서 LKSangle 초기값 확인
```python
LKSangle = 0.0  # ✅ 0으로 초기화
```

### 3️⃣ **stear_vision.py 파라미터 불일치**
**상태**: ⚠️ **심각도: 낮음**

```python
# stear_vision.py::main()
def main():
    pass
```

main.py에서 호출:
```python
stearVisionThread = threading.Thread(target=stear_main, args=(stop_event, args))
```

**문제**: `stear_vision.py::main()`이 `stop_event` 파라미터를 사용하지 않음

**해결**: main() 함수 서명 수정 필요
```python
def main(stop_event=None, args=None):
    while not (stop_event and stop_event.is_set()):
        ...
```

### 4️⃣ **motorControl 예외 처리 부족**
**상태**: ⚠️ **심각도: 낮음**

```python
# motorControl.py
def lonControl(vel):
    vel = max(min(vel, 100), -100)  # None 체크 없음
```

**해결**: None 체크 추가
```python
def lonControl(vel):
    if vel is None:
        vel = 0
    vel = max(min(vel, 100), -100)
    ...
```

### 5️⃣ **동시성 문제 (Race Condition)**
**상태**: ⚠️ **심각도: 중간**

여러 스레드가 globalVar를 동시에 수정할 수 있음:
- stearVisionThread: `LKSangle` 쓰기
- mainTaskThread: `desiredSpeed` 쓰기
- 메인 스레드: `userTargetSpeed`, `userTargetAngle` 쓰기

**해결**: Lock 추가 (예시)
```python
import threading
globalVar_lock = threading.Lock()

# stear_vision.py
with globalVar_lock:
    globalVar.LKSangle = steering_cmd
```

### 6️⃣ **에러 복구 미흡**
**상태**: ⚠️ **심각도: 중간**

motorControl 함수 호출 실패 시 대응책 없음

**해결**: try-except 추가
```python
try:
    lonControl(final_speed)
    latControl(final_angle)
except Exception as e:
    logger.error(f"Motor control error: {e}")
```

---

## ✅ 검증 체크리스트

- [x] main.py에서 3개 스레드 정상 시작
- [x] globalVar 변수 초기화 확인
- [x] stear_vision.py 임포트 확인
- [x] taskManager.py 함수 존재 확인
- [ ] ⚠️ globalVar.userTargetSpeed 초기화 필요
- [ ] ⚠️ globalVar.userTargetAngle 초기화 필요
- [ ] ⚠️ stear_vision.py::main() 파라미터 수정 필요
- [ ] ⚠️ Race condition 해결 필요
- [ ] ⚠️ motorControl 예외 처리 강화 필요

---

## 🔧 권장 수정 순서

1. **globalVar.py 수정** (필수)
   ```python
   userTargetSpeed = 0
   userTargetAngle = 0
   ```

2. **stear_vision.py::main() 수정** (권장)
   ```python
   def main(stop_event=None, args=None):
       ...
       while not (stop_event and stop_event.is_set()):
   ```

3. **motorControl.py 강화** (권장)
   - None 체크 추가
   - 예외 처리 추가

4. **Race condition 해결** (선택)
   - Lock 추가 또는
   - Queue 사용

---

## 📈 데이터 흐름 요약

```
┌─────────────────────────────────────────────────────┐
│           main.py (메인 스레드 + 3개 워커)          │
├─────────────────────────────────────────────────────┤
│                                                      │
│  stearVisionThread          objInfoTaskThread       │
│  ┌──────────────────┐      ┌─────────────────┐     │
│  │ 차선 추적 (LKS)  │      │ 객체 & 구역 감지 │     │
│  │ LKSangle ←────────→ globalVar ←────────────     │
│  └──────────────────┘      └─────────────────┘     │
│         ↓                           ↓               │
│       zoneInfo                      │               │
│         ↓                           ↓               │
│  ┌──────────────────────────────────────────┐      │
│  │    mainTaskThread (속도 제어)            │      │
│  │    desiredSpeed 계산 & 모터 제어         │      │
│  └──────────────────────────────────────────┘      │
│         ↓                                           │
│    motorControl API                                 │
│    (lonControl, latControl)                         │
│         ↓                                           │
│    ┌──────────────┐        ┌──────────────┐        │
│    │  Motor/PWM   │        │   Servo      │        │
│    │  (속도)      │        │   (조향)     │        │
│    └──────────────┘        └──────────────┘        │
│                                                      │
└─────────────────────────────────────────────────────┘
```

---

## 🎯 결론

**현재 상태**: 대부분 정상 작동하지만 **필수 초기화 부분 누락**

**즉시 수정 필요**:
1. globalVar.userTargetSpeed 초기화
2. globalVar.userTargetAngle 초기화

**권장 수정**:
1. stear_vision.py::main() 파라미터
2. motorControl 예외 처리

**선택 개선**:
1. Race condition 방지
2. 상세 로깅 추가
