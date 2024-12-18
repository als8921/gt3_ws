import open3d as o3d
import numpy as np

pcl_path = 'pcl_visualization/pointcloud_data/pointcloud_data_0.txt'

def load_point_cloud(pcl_path):
    data = np.loadtxt(pcl_path)
    return data

points = load_point_cloud(pcl_path)

# Open3D 포인트 클라우드 객체 생성
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Voxel Filtter
downpcd = pcd.voxel_down_sample(voxel_size = 0.01)

# Normal 벡터 추정
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30)
)

if(downpcd.has_normals()):
    print("Normal Vector 가 있음")
else:
    print("Normal Vector 가 없음")


# Open3D 시각화
o3d.visualization.draw_geometries([downpcd], point_show_normal=True)
