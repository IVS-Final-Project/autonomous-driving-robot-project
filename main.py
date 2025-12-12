import threading
import time
import sys
import termios
import tty
import globalVar
from taskManager2 import getObjInfoTask, mainTask
from motorControl import lonControl, latControl

# 키보드 설정값
SPEED_STEP = 10
ANGLE_STEP = 15

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if sys.stdin.readable():
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == "__main__":
    stop_event = threading.Event()
    args = {}

    # 스레드 시작
    objInfoTaskThread = threading.Thread(target=getObjInfoTask, args=(stop_event, args))
    mainTaskThread = threading.Thread(target=mainTask, args=(stop_event, args))
    
    objInfoTaskThread.start()
    mainTaskThread.start()

    print("=== Manual Control Mode ===")
    print(" W/S: Speed Up/Down")
    print(" A/D: Steering Left/Right")
    print(" Q: Quit")

    try:
        while True:
            # 1. 키보드 입력 처리
            key = get_key()
            if key == 'w':
                globalVar.userTargetSpeed += SPEED_STEP
                if globalVar.userTargetSpeed > 100: globalVar.userTargetSpeed = 100
                print(f"[USER] Speed UP -> {globalVar.userTargetSpeed}")
            elif key == 's':
                globalVar.userTargetSpeed -= SPEED_STEP
                if globalVar.userTargetSpeed < -100: globalVar.userTargetSpeed = -100
                print(f"[USER] Speed DOWN -> {globalVar.userTargetSpeed}")
            elif key == 'a':
                globalVar.userTargetAngle -= ANGLE_STEP
                if globalVar.userTargetAngle < -90: globalVar.userTargetAngle = -90
                print(f"[USER] Left -> {globalVar.userTargetAngle}")
            elif key == 'd':
                globalVar.userTargetAngle += ANGLE_STEP
                if globalVar.userTargetAngle > 90: globalVar.userTargetAngle = 90
                print(f"[USER] Right -> {globalVar.userTargetAngle}")
            elif key == 'q':
                break

            # 2. 모터 제어 (taskManager2에서 계산된 최종 값 사용)
            final_speed = globalVar.desiredSpeed
            
            # 조향은 사용자가 직접 제어
            globalVar.desiredAngle = globalVar.userTargetAngle 
            final_angle = globalVar.desiredAngle

            lonControl(final_speed)
            latControl(final_angle)

            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
        
    finally:
        stop_event.set()
        objInfoTaskThread.join()
        mainTaskThread.join()
        print("System Stopped.")
