from pointcloud_data.cluster_data import data
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def get_normal_vector(y, x, data):
    """주어진 (y, x) 위치의 법선 벡터를 계산하는 함수."""
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


def get_normal_vectors(data):
    """주어진 포인트 클라우드 데이터의 모든 법선 벡터를 계산하는 함수."""
    rows, cols, _ = data.shape
    normals = np.zeros((rows, cols, 3))

    for y in range(1, rows - 1):
        for x in range(1, cols - 1):
            normals[y, x] = get_normal_vector(y, x, data)

    return normals

def visualize_normals(data, normals):
    """포인트 클라우드와 법선 벡터를 시각화하는 함수."""
    fig = plt.figure(figsize=(16, 16))
    ax = fig.add_subplot(111, projection='3d')

    # 포인트 플롯
    ax.scatter(data[:, :, 0], data[:, :, 1], data[:, :, 2], color='b', label='Points')

    # 법선 벡터 플롯
    for y in range(1, data.shape[0] - 1):
        for x in range(1, data.shape[1] - 1):
            # 법선 벡터 시작점
            start = data[y, x]
            # 법선 벡터 끝점
            ax.quiver(start[0], start[1], start[2],
                      normals[y, x, 0], normals[y, x, 1], normals[y, x, 2],
                      length=0.3, arrow_length_ratio = 0.1, color='r')

    # 축 레이블 설정
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Points and Normals')
    ax.legend()

    # 스케일을 같게 설정
    x_max = np.nanmax(data[:, :, 0])
    x_min = np.nanmin(data[:, :, 0])
    y_max = np.nanmax(data[:, :, 1])
    y_min = np.nanmin(data[:, :, 1])
    z_max = np.nanmax(data[:, :, 2])
    z_min = np.nanmin(data[:, :, 2])

    max_range = np.array([x_max - x_min, y_max - y_min, z_max - z_min]).max() / 2.0

    mid_x = (x_max + x_min) * 0.5
    mid_y = (y_max + y_min) * 0.5
    mid_z = (z_max + z_min) * 0.5

    # 축의 범위 설정
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

# 메인 실행 부분
normals = get_normal_vectors(data)
visualize_normals(data, normals)
