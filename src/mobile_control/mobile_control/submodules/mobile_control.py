import math
posX, posY, theta = 0, 0, 0
def NormalizeAngle(angle):
    return (angle + 180) % 360 - 180


def DirectionConversion(x, y, theta):
    """
    전/후진을 전환하는 부분
    회전은 그대로
    목표 각도값이 있다면 로봇에게는 +180도를 하고 normalize를 한 각도를 명령으로 줘야한다.
    """
    pass


def ControlMission(x_d, y_d, theta_d):
    """
    X, Y, Theta 명령을 받았을 때 지령을 생성하기
    1. 목표 좌표를 바라보기
    2. 목표 좌표까지 이동
    3. 목표 각도로 회전
    """
    Rotate(NormalizeAngle(math.degrees(math.atan2(y_d - posY, x_d - posX))))
    MoveForward(math.sqrt((posX - x_d)**2 + (posY - y_d)**2))
    Rotate(NormalizeAngle(theta_d))
    pass


def MoveForward(distance):
    print("Distance : ", distance)
    pass

def Rotate(theta):
    print("Theta : ", theta)
    pass

ControlMission(1, -1, 90)