import numpy as np
from collections import deque

def CreateCommandPositionQueue(x1, y1, x2, y2, D_horizontal, D_vertical, D_task):
    # (x2 - x1, y2 - y1) 벡터 정의 및 정규화
    move_vector = np.array([x2 - x1, y2 - y1], dtype=float)
    move_d = np.linalg.norm(move_vector)
    move_vector_norm = move_vector / move_d
    
    # 외적 방향 벡터 계산
    z_unit_vector = np.array([0, 0, 1], dtype=float)
    direction_vector = np.cross(move_vector, z_unit_vector)[:2]
    direction_vector = direction_vector / np.linalg.norm(direction_vector)  # 정규화
    
    # 새로운 위치 계산
    new_pos = np.array([x1, y1], dtype=float) + move_vector_norm * D_horizontal + direction_vector * D_vertical
    
    # theta 계산 (외적 방향의 각도)
    theta_degrees = np.degrees(np.arctan2(-direction_vector[1], -direction_vector[0]))

    queue = deque()
    
    for i in range(0, int((move_d - 2 * D_horizontal) // D_task) + 1):
        current_position = new_pos + move_vector_norm * (D_task * i)
        queue.append((current_position[0], current_position[1], theta_degrees))

    # 마지막 위치 추가 (모듈로 연산으로 인한 위치)
    if (move_d - 2 * D_horizontal) % D_task != 0:
        last_position = new_pos + move_vector_norm * (move_d - 2 * D_horizontal)
        queue.append((last_position[0], last_position[1], theta_degrees))
    
    return queue

# 사용 예시
x1, y1 = 1, 0
x2, y2 = 5, 0
D_horizontal = 1  # x 방향 거리
D_vertical = 0.5  # 외적 방향 거리
D_task = 1.2  # 작업 거리

queue = CreateCommandPositionQueue(x1, y1, x2, y2, D_horizontal, D_vertical, D_task)

# 큐 내용 출력
print("새로운 위치 (x, y, theta):")
for position in queue:
    print(position)
