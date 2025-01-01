import numpy as np

def rotate_point(point, a, b, c):
    # 각도를 라디안으로 변환
    a_rad = np.radians(a)
    b_rad = np.radians(b)
    c_rad = np.radians(c)

    # 회전 행렬 정의
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(a_rad), -np.sin(a_rad)],
                   [0, np.sin(a_rad), np.cos(a_rad)]])

    Ry = np.array([[np.cos(b_rad), 0, np.sin(b_rad)],
                   [0, 1, 0],
                   [-np.sin(b_rad), 0, np.cos(b_rad)]])

    Rz = np.array([[np.cos(c_rad), -np.sin(c_rad), 0],
                   [np.sin(c_rad), np.cos(c_rad), 0],
                   [0, 0, 1]])

    # 전체 회전 행렬
    R = Rz @ Ry @ Rx  # ZYX 순서로 회전

    # 점 회전
    rotated_point = R @ point
    return rotated_point, R

def angles_from_rotated_point(rotated_point, R):
    # 회전 행렬의 역행렬
    R_inv = np.linalg.inv(R)

    # 원래의 점을 복원
    original_point = R_inv @ rotated_point

    # 원래의 각도 계산
    a = np.degrees(np.arctan2(original_point[2], original_point[1]))  # x축 회전
    b = np.degrees(np.arctan2(-original_point[0], np.sqrt(original_point[1]**2 + original_point[2]**2)))  # y축 회전
    c = np.degrees(np.arctan2(original_point[1], original_point[0]))  # z축 회전

    return a, b, c

# 초기 점
point = np.array([1, 0, 0])

# 회전 각도 (예: x=-150도, y=-35도, z=-50도)
a = 0  # x축 회전
b = 45   # y축 회전
c = 0   # z축 회전

# 회전된 점 및 회전 행렬 계산
rotated_point, R = rotate_point(point, a, b, c)
x0, y0, z0 = rotated_point

print(f"회전된 좌표: x0 = {x0}, y0 = {y0}, z0 = {z0}")

# 원래의 각도 계산
angles = angles_from_rotated_point(rotated_point, R)
print(f"계산된 각도: a = {angles[0]}, b = {angles[1]}, c = {angles[2]}")



a = angles[0]  # x축 회전
b = angles[1]   # y축 회전
c = angles[2]   # z축 회전


rotated_point, R = rotate_point(np.array([1, 0, 0]), a, b, c)
x0, y0, z0 = rotated_point

print(f"회전된 좌표: x0 = {x0}, y0 = {y0}, z0 = {z0}")
