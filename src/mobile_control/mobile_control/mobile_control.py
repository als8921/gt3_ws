def normalize_angle(angle):
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
    pass