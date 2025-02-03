from .parameters import *
import math
class Gear:
    Disable = 0
    Parking = 1
    Neutral = 2
    Differential = 6
    Lateral = 8
    Rotate = 10

class Mode:
    NONE = 0
    PaintMode = 1
    FinalArrived = 2
    ScanMode = 3
    NextMove = 4

class Position:
    def __init__(self):
        self.x = 0.0        # [m]
        self.y = 0.0        # [m]
        self.theta = 0.0    # [deg]

class Command(Position):
    def __init__(self):
        super().__init__()
        self.gearSetting = Gear.Differential
        self.height = 0.0
        self.mode = 0

def NormalizeAngle(angle):
    return (angle + 180) % 360 - 180

def AngularSpeedLimit(speed, maxSpeed = MaxAngularSpeed):
    return float(max(-maxSpeed, min(speed, maxSpeed)))

def LinearXSpeedLimit(speed, minSpeed = MinXLinearSpeed, maxSpeed = MaxXLinearSpeed):
    return float(max(minSpeed, min(speed, maxSpeed)))

def LinearYSpeedLimit(speed, minSpeed = -MaxYLinearSpeed, maxSpeed = MaxYLinearSpeed):
    return float(max(minSpeed, min(speed, maxSpeed)))

def RelativeDistance(a : Position, b : Position):
    return math.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)

def RelativeAngle(a : Position, b : Position):
    return NormalizeAngle(math.degrees(math.atan2(b.y - a.y, b.x - a.x)))