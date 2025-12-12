from gpiozero import AngularServo, Motor
from time import sleep


servoPWM = 17
MotorENA = 4
MotorIN1 = 21
MotorIN2 = 26

servo = AngularServo(servoPWM, min_angle=-90, max_angle=90,
                     min_pulse_width=0.0005, max_pulse_width=0.0024)
rear_motor = Motor(forward=MotorIN1, backward=MotorIN2, enable=MotorENA, pwm=True)


def lonControl(vel):
    if vel is None:
        vel = 0
    vel = max(min(vel, 100), -100)
    print(f"Speed: {vel}")
    vel /= 100
    
    if vel > 0:
        rear_motor.forward(vel)
    elif vel == 0:
        rear_motor.stop()
    else:
        rear_motor.backward(-vel)
    

def latControl(angle):
    if angle is None:
        angle = 0
    angle = max(min(angle, 90), -90)
    
    #print(f"Steering Angle: {angle}")
    servo.angle = angle + 2


