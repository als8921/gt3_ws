import numpy as np
import math
import matplotlib.pyplot as plt

class Position:
    def __init__(self):
        self.x = 0.0        # [m]
        self.y = 0.0        # [m]
        self.theta = 0.0    # [deg]

def NormalizeAngle(angle):
    return (angle + 180) % 360 - 180
def RelativeAngle(a : Position, b : Position):
    return NormalizeAngle(math.degrees(math.atan2(b.y - a.y, b.x - a.x)))

x1, y1 = -3, 1
x2, y2 = -3, 2

interPosition = Position()

if x2 - x1 == 0:
    interPosition.x = x1
    interPosition.y = 0

elif y1 - y2 == 0:
    interPosition.x = 0
    interPosition.y = y1

else:
    m = (y2 - y1) / (x2 - x1)
    n = y1 - m * x1
    interPosition.x = n / ((-1 / m) - m)
    interPosition.y = (-1 / m) * interPosition.x

print(f"교점: ({interPosition.x:.2f}, {interPosition.y:.2f})")
print(RelativeAngle(Position(), interPosition))

plt.figure(figsize=(12, 12))
plt.scatter([x1, x2], [y1, y2], color='green', label='Points (x1, y1) and (x2, y2)')
plt.plot([0, interPosition.x],[0, interPosition.y], 'r-')

plt.xlim(-5, 5)
plt.ylim(-5, 5)
plt.axhline(0, color='black', linewidth=0.5, ls='--')
plt.axvline(0, color='black', linewidth=0.5, ls='--')
plt.grid()
plt.legend()
plt.title('Intersection of Lines')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.show()
