import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

def cluster_pointcloud(pointcloud):
    temp_points = []
    for x, y, z in pointcloud:
        if(x != 0 and y != 0 and z !=0):
            temp_points.append([x, y, z])

    data = np.array(temp_points)
    # 데이터 스케일링
    scaler = StandardScaler()
    data_scaled = scaler.fit_transform(data)

    # DBSCAN 클러스터링
    dbscan = DBSCAN(eps=0.2, min_samples=10)  # eps와 min_samples 조정
    labels = dbscan.fit_predict(data_scaled)

    # 클러스터 개수 (노이즈는 -1로 레이블링됨)
    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print(f'발견된 클러스터 수: {num_clusters}')

    # 0, 0, 0에 가장 가까운 클러스터 찾기
    origin = np.array([0, 0, 0])
    closest_cluster_label = None
    closest_distance = float('inf')

    for label in set(labels):
        if label != -1:  # 노이즈 포인트는 제외
            # 해당 클러스터의 데이터 추출
            cluster_data = data[labels == label]
            # 클러스터의 중심 계산
            cluster_center = np.mean(cluster_data, axis=0)
            # 0, 0, 0과의 거리 계산
            distance = np.linalg.norm(cluster_center - origin)
            if distance < closest_distance:
                closest_distance = distance
                closest_cluster_label = label

    # 가장 가까운 클러스터 데이터와 나머지 데이터 추출
    closest_cluster_data = data[labels == closest_cluster_label]
    remaining_data = data[labels != closest_cluster_label]

    return closest_cluster_data.tolist(), remaining_data.tolist()

# 예시 사용
# file_path = 'pcl_visualization/pointcloud_data/pointcloud_data_0.txt'
# closest, remaining = cluster_pointcloud(np.loadtxt(file_path))

# print("가장 가까운 클러스터 데이터:")
# print(closest)
# print("나머지 데이터:")
# print(remaining)
