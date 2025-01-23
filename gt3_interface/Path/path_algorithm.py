import numpy as np

def get_normal_vector(y, x, data):
    """주어진 (y, x) 위치의 법선 벡터를 계산합니다.

    Args:
        y (int): y 좌표.
        x (int): x 좌표.
        data (numpy.ndarray): 포인트 클라우드 데이터.

    Returns:
        numpy.ndarray: (x, y, z) 형식의 법선 벡터.
    """
    cross_vector = np.zeros(3)

    # 5x5 영역으로 법선 벡터 계산
    for i in (-2, -1, 0, 1, 2):
        for j in (-2, -1, 0, 1, 2):
            ny = y + j
            nx = x + i

            # 유효한 인덱스인지 확인
            if 0 <= ny < data.shape[0] and 0 <= nx < data.shape[1]:
                center = data[y, x]
                up = data[ny - 1, nx] if ny > 0 else center
                down = data[ny + 1, nx] if ny < data.shape[0] - 1 else center
                left = data[ny, nx - 1] if nx > 0 else center
                right = data[ny, nx + 1] if nx < data.shape[1] - 1 else center

                # 각 벡터 계산
                up1 = np.array(up) - np.array(center)
                right1 = np.array(right) - np.array(center)
                down1 = np.array(down) - np.array(center)
                left1 = np.array(left) - np.array(center)

                # 유효한 벡터 확인 후 외적 수행
                if not np.isnan(up1).any() and not np.isnan(right1).any():
                    cross_vector += np.cross(right1, up1)

                if not np.isnan(up1).any() and not np.isnan(left1).any():
                    cross_vector += np.cross(up1, left1)

                if not np.isnan(left1).any() and not np.isnan(down1).any():
                    cross_vector += np.cross(left1, down1)

                if not np.isnan(down1).any() and not np.isnan(right1).any():
                    cross_vector += np.cross(down1, right1)

    # 법선 벡터 정규화
    if np.linalg.norm(cross_vector) != 0:
        cross_vector /= np.linalg.norm(cross_vector)

    return cross_vector

def get_start_end_index(clust):
    """
    클러스터에서 시작 및 종료 인덱스를 찾는 함수

    Args:
        clust: 클러스터 데이터 shape가 (height, width, 3)인 np.array

    Returns:
        각 행에 대한 시작 및 종료 인덱스의 리스트
    """
    clust = np.array(clust)
    result = []
    for y in range(clust.shape[0]):
        start_point = -1
        end_point = -1

        for i in range(clust.shape[1]):
            if(start_point == -1 and not np.isnan(clust[y][i]).any()):
                start_point = i
            if(end_point == -1 and not np.isnan(clust[y][clust.shape[1] - 1 - i]).any()):
                end_point = clust.shape[1] - 1 - i
        result.append([y, start_point, end_point])

    return result

def create_path(clust, length):
    """
    시작 및 종료 위치와 방향 벡터를 반환하는 함수

    Args:
        clust: 클러스터 데이터 (2D 배열)
        length: 시작, 끝점으로 부터 추가 이동 길이

    Returns:
        [y (몇 번째 줄인지), [X_s, Y_s, Z_s, dx_s, dy_s, dz_s], [X_e, Y_e, Z_e, dx_e, dy_e, dz_e]] 형태의 데이터를 원소로 하는 리스트
    """
    def normalize(vector):
        norm = np.linalg.norm(vector)
        return vector / norm if norm != 0 else vector
    result = []
    clust = np.array(clust)
    start_end_idx = get_start_end_index(clust)
    for y, start, end in start_end_idx:
        if(end - start >= 2):
            startPos = clust[y][start] + normalize(clust[y][start] - clust[y][start + 1]) * length
            startDir = get_normal_vector(y, start, clust)
            endPos = clust[y][end] + normalize(clust[y][end] - clust[y][end - 1]) * length
            endDir = get_normal_vector(y, end, clust)
            result.append([y, [*startPos, *startDir], [*endPos, *endDir]])

    return result
            