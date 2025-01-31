from enum import Enum
class State(Enum):
    Stop = "Stop"
    InitialRotate = 0
    MoveForward = 1
    MoveLateral = 2
    FinalRotate = 3
    ScanRotate = 4
