import numpy as np
from copy import deepcopy
def correction(data):
    """
    PointCloud Clust 데이터의 빈 값을 보정하여 리턴합니다.
    
    이 함수는 주어진 3차원 PointCloud 클러스터 데이터에서 NaN 값을 찾아 보정합니다.
    가로 및 세로 방향으로 NaN 값을 보정한 후, 두 보정값 중 NaN이 아닌 값을 선택하여 최종적으로 리턴합니다.
    
    Args:
        data (np.array): 클러스터 데이터, shape은 (width, height, 3)입니다.
                         각 요소는 [x, y, z] 좌표를 나타냅니다.
    
    Returns:
        np.array: 보정된 클러스터 데이터, shape은 (width, height, 3)입니다.
                  NaN 값이 보정되어 있습니다.
    """
    correctionHorizontal = np.array(deepcopy(data))
    correctionVertical = np.array(deepcopy(data))
    final_correction = np.array(deepcopy(data))

    for y in range(correctionHorizontal.shape[0]):
        nan_count = 0
        for x in range(correctionHorizontal.shape[1]):
            if np.isnan(correctionHorizontal[y][x]).any():
                nan_count += 1
                
            else:
                if nan_count > 0 and nan_count < 10:
                    start_idx = x - nan_count
                    end_idx = x - 1
                    
                    left_value = correctionHorizontal[y][start_idx - 1]
                    right_value = correctionHorizontal[y][x]
                    
                    if not np.isnan(left_value).any() and not np.isnan(right_value).any():

                        fill_values = np.linspace(left_value, right_value, nan_count + 2)[1:-1]
                        correctionHorizontal[y][start_idx:end_idx + 1] = fill_values

                nan_count = 0

    for x in range(correctionVertical.shape[1]):
        nan_count = 0
        for y in range(correctionVertical.shape[0]):
            if np.isnan(correctionVertical[y][x]).any():
                nan_count += 1
                
            else:
                if nan_count > 0 and nan_count < 10:
                    start_idx = y - nan_count
                    end_idx = y - 1
                    
                    left_value = correctionVertical[start_idx - 1][x] if start_idx > 0 else np.nan
                    right_value = correctionVertical[y][x]
                    
                    if not np.isnan(left_value).any() and not np.isnan(right_value).any():
                        fill_values = np.linspace(left_value, right_value, nan_count + 2)[1:-1]
                        correctionVertical[start_idx:end_idx + 1, x] = fill_values

                nan_count = 0

    for y in range(final_correction.shape[0]):
        for x in range(final_correction.shape[1]):
            if np.isnan(data[y][x]).any():
                horizontal_value = correctionHorizontal[y][x]
                vertical_value = correctionVertical[y][x]

                if not np.isnan(horizontal_value).any() and np.isnan(vertical_value).any():
                    final_correction[y][x] = horizontal_value
                elif np.isnan(horizontal_value).any() and not np.isnan(vertical_value).any():
                    final_correction[y][x] = vertical_value
                elif not np.isnan(horizontal_value).any() and not np.isnan(vertical_value).any():
                    if(horizontal_value[0] > vertical_value[0]):
                        final_correction[y][x] = vertical_value
                    else:
                        final_correction[y][x] = horizontal_value

    return final_correction


if __name__ == "__main__":
    from pointcloud_data.cluster_data import data
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import pcl_normal_vector

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

    data = correction(data)

    normals = pcl_normal_vector.get_normal_vectors(data)
    visualize_normals(data, normals)