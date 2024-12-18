import open3d as o3d
import numpy as np

# 파일 경로
file_path = 'pcl_visualization/pointcloud_data/pointcloud_data_0.txt'

# 데이터를 읽어오는 함수
def load_point_cloud(file_path):
    # 파일에서 데이터를 읽어오기
    data = np.loadtxt(file_path)
    return data

# 포인트 클라우드 데이터 로드
points = load_point_cloud(file_path)

# Open3D 포인트 클라우드 객체 생성
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# 포인트 클라우드 시각화
o3d.visualization.draw_geometries([pcd])
