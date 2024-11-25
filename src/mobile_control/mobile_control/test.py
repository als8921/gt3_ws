import numpy as np

def calculate_new_position_and_theta(x1, y1, x2, y2, alpha, beta):
    # (x2 - x1, y2 - y1) 벡터 정의 및 정규화
    move_vector = np.array([x2 - x1, y2 - y1], dtype=float)
    move_vector_norm = move_vector / np.linalg.norm(move_vector)
    
    # 외적 방향 벡터 계산
    z_unit_vector = np.array([0, 0, 1], dtype=float)
    direction_vector = np.cross(move_vector, z_unit_vector)[:2]
    direction_vector = direction_vector / np.linalg.norm(direction_vector)  # 정규화
    
    # 새로운 위치 계산
    new_pos = np.array([x1, y1], dtype=float) + move_vector_norm * alpha + direction_vector * beta
    
    # theta 계산 (외적 방향의 각도)
    theta_degrees = np.degrees(np.arctan2(-direction_vector[1], -direction_vector[0]))
    
    return new_pos, theta_degrees

# 사용 예시
x1, y1 = 1, 0
x2, y2 = 5, 0
alpha = 1  # 방향 벡터의 크기
beta = 0.5  # 외적 방향 벡터의 크기

new_position, theta = calculate_new_position_and_theta(x1, y1, x2, y2, alpha, beta)

print("새로운 위치 (x, y):", new_position)
print("theta:", theta)
