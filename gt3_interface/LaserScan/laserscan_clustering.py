import numpy as np
from sklearn.cluster import DBSCAN

def clustering(x, y, eps, min_samples):
    """DBSCAN 알고리즘을 사용하여 클러스터링을 수행합니다.

    Args:
        x (list): x 좌표 값의 리스트.
        y (list): y 좌표 값의 리스트.
        eps (float): 클러스터링의 밀도 기준.
        min_samples (int): 최소 샘플 수.

    Returns:
        list: 각 클러스터의 x, y 좌표 리스트를 포함하는 리스트.
    """
    data = np.array(list(zip(x, y)))
    
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(data)
    labels = db.labels_
    clusters = []
    
    unique_labels = set(labels)
    for label in unique_labels:
        if label != -1:  # -1은 노이즈를 의미
            cluster_indices = np.where(labels == label)[0]
            cluster_x = [x[i] for i in cluster_indices]
            cluster_y = [y[i] for i in cluster_indices]
            clusters.append([cluster_x, cluster_y])
    
    def closest_distance(cluster):
        """주어진 클러스터의 점들 중 (0, 0)과의 가장 가까운 거리 계산.

        Args:
            cluster (list): 클러스터의 x, y 좌표 리스트.

        Returns:
            float: (0, 0)과의 최소 거리.
        """
        distances = [np.sqrt(cx**2 + cy**2) for cx, cy in zip(cluster[0], cluster[1])]
        return min(distances)

    # 클러스터를 (0, 0)과의 거리로 정렬
    clusters.sort(key=closest_distance)

    return clusters
