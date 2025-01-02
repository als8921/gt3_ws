import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

def cluster_pointcloud(origin_pos, pointcloud, _eps):
    print(origin_pos)
    temp_points = []
    original_indices = []  # 원래 인덱스를 저장할 리스트 추가
    for idx, (x, y, z) in enumerate(pointcloud):
        if x != 0 and y != 0 and z != 0:
            temp_points.append([x, y, z])
            original_indices.append(idx)  # 원래 인덱스 추가

    data = np.array(temp_points)
    # 데이터 스케일링
    scaler = StandardScaler()
    data_scaled = scaler.fit_transform(data)

    # DBSCAN 클러스터링
    dbscan = DBSCAN(eps=_eps, min_samples=10)  # eps와 min_samples 조정
    labels = dbscan.fit_predict(data_scaled)

    # 클러스터 개수 (노이즈는 -1로 레이블링됨)
    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print(f'발견된 클러스터 수: {num_clusters}')

    # origin_pos에 가장 가까운 클러스터 찾기
    origin = np.array(origin_pos)
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

    # closest_cluster_data의 인덱스 찾기 (원래 인덱스 기반)
    closest_index = [original_indices[i] for i in np.where(labels == closest_cluster_label)[0].tolist()]

    width, height = 43, 24
    image = np.ones((height, width, 3))
    if(len(pointcloud) <= 1032):
        for index in closest_index:
            y = index // width
            x = index % width
            image[y][x] = [0, 0, 0]

    return closest_cluster_data.tolist(), remaining_data.tolist(), closest_index, image