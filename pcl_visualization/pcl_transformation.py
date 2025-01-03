import numpy as np

def transform_points(points, end_effector_position):
    """주어진 점들을 엔드 이펙터의 위치와 방향에 따라 변환합니다.

    Args:
        points (list of tuple): 변환할 3D 점들의 리스트입니다.
        end_effector_position (tuple): 엔드 이펙터의 위치와 방향을 포함하는 튜플로,
            형식은 (x, y, z, roll, pitch, yaw)입니다.
    
    Returns:
        list: 변환된 3D 점들의 리스트입니다.
    """
    end_x, end_y, end_z, end_r, end_p, end_yaw = end_effector_position

    rotation_matrix = get_rotation_matrix(end_r, end_p, end_yaw)
    transformed_points = []

    for point in points:
        if point == (0, 0, 0):
            transformed_points.append([0.0, 0.0, 0.0])
        else:
            rotated_point = rotation_matrix @ np.array(point)
            transformed_point = rotated_point + np.array([end_x, end_y, end_z])
            transformed_points.append(transformed_point)

    return transformed_points

def get_rotation_matrix(roll, pitch, yaw):
    """주어진 roll, pitch, yaw 각도에 대한 회전 행렬을 계산합니다.

    Args:
        roll (float): 롤 각도(라디안 단위)입니다.
        pitch (float): 피치 각도(라디안 단위)입니다.
        yaw (float): 요 각도(라디안 단위)입니다.
    
    Returns:
        numpy.ndarray: 3x3 회전 행렬입니다.
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    return R_z @ R_y @ R_x
