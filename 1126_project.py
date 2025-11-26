#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# EV3 본체
ev3 = EV3Brick()

# 방향 상수 (필요하면 나중에 사용)
DIR_N = 1
DIR_E = 2
DIR_S = 3
DIR_W = 4

# 모터 & 센서 설정
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
grab_motor = Motor(Port.B)

left_color = ColorSensor(Port.S1)
right_color = ColorSensor(Port.S4)
jcolor = ColorSensor(Port.S2)      # 집은 물체 색 감지용
ultra = UltrasonicSensor(Port.S3)

# 로봇 베이스 (wheel_diameter, axle_track 값은 상황 맞게 조절)
robot = DriveBase(left_motor, right_motor, 55.5, 104)

# 라인트레이싱 기본 파라미터
THRESHOLD = 50
KP = 1.2


# -------------------------
# 집게 모터 제어
# -------------------------
def release_object():
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)


def grab_object():
    grab_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)


# -------------------------
# 라인트레이싱 한 스텝 (좌측 기준)
# -------------------------
def line_follow_left_step(speed, kp):
    reflection = left_color.reflection()
    error = reflection - THRESHOLD
    turn_rate = kp * error
    robot.drive(speed, turn_rate)


# -------------------------
# 라인트레이싱 한 스텝 (우측 기준)
# -------------------------
def line_follow_right_step(speed, kp):
    reflection = right_color.reflection()
    error = reflection - THRESHOLD
    turn_rate = -kp * error
    robot.drive(speed, turn_rate)


# -------------------------
# 1) 물체를 찾을 때까지 라인 따라가기 + 잡기
#    → 물체를 잡으면 함수가 끝나도록 설계
# -------------------------
def move_until_object(direction="left", dist_threshold=50):
    """
    direction: 'left' 또는 'right'
    dist_threshold: 초음파 센서 거리 조건 (mm 단위)
    """
    while True:
        # 1. 물체 감지
        if ultra and ultra.distance() < dist_threshold:
            ev3.speaker.beep()
            robot.stop()
            # 물체 쪽으로 살짝 더 전진 (상황에 맞게 조절)
            robot.straight(100)
            grab_object()
            robot.stop()
            break  # ★ 여기서 함수 종료

        # 2. 라인 따라가기
        if direction == "left":
            line_follow_left_step(100, KP)
        else:
            line_follow_right_step(100, KP)

        wait(10)  # 센서 업데이트 시간을 조금 줌


# -------------------------
# 2) n칸 이동 (물체 감지 없이 라인만 따라감)
#    “교차로/칸” 개념이라면 흰 → 검정 → 흰 패턴 기준
# -------------------------
def n_move(n, direction="right"):
    for _ in range(n):
        if direction == "right":
            # 흰 바탕에서 라인을 찾을 때까지
            while right_color.reflection() > THRESHOLD:
                line_follow_left_step(100, KP)
                wait(10)

            # 라인(검정)을 따라갈 때
            while right_color.reflection() <= THRESHOLD:
                line_follow_right_step(100, KP)
                wait(10)

        elif direction == "left":
            while left_color.reflection() > THRESHOLD:
                line_follow_right_step(100, KP)
                wait(10)

            while left_color.reflection() <= THRESHOLD:
                line_follow_left_step(100, KP)
                wait(10)

    robot.stop()


# -------------------------
# 메인 동작
# -------------------------
def start():
    # 혹시 집게에 뭔가 잡혀 있으면 먼저 풀기
    release_object()

    # 1) 물체를 찾을 때까지 라인트레이싱 (물체를 잡으면 함수 종료)
    #    → 여기서는 'left' 기준으로 라인 따라가면서 초음파로 물체 찾기
    move_until_object(direction="left", dist_threshold=50)

    # 2) 물체 색 인식
    wait(100)
    object_color = jcolor.rgb()
    print("Detected object color: ", object_color)

    # 3) 색에 따라 다른 경로로 이동해서 물체 내려놓기
    #    이제부터는 물체 감지 필요 없으니 n_move만 사용

    # red 물체 처리 (R 채널이 크면)
    #object_color[0] > 100:
    robot.turn(180)
    robot.straight(30)
    n_move(1, direction="left")
    robot.straight(50)
    robot.turn(-90)
    robot.straight(30)
    n_move(1, direction="right")
    robot.turn(90)
    robot.straight(30)
    n_move(1, direction="right")
    release_object()

    # blue 물체 처리 (B 채널이 크면)
    # elif object_color[2] > 100:
    #     robot.turn(180)
    #     n_move(1, direction="right")
    #     robot.turn(-90)
    #     n_move(2, direction="left")
    #     robot.turn(90)
    #     n_move(1, direction="left")
    #     release_object()

    # 그 외 색이면 일단 그냥 멈추기
    # else:
    #     ev3.speaker.beep()
    #     robot.stop()


# 프로그램 시작
start()
