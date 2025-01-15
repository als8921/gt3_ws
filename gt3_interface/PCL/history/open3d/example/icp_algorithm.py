import open3d as o3d
import copy
import numpy as np


def load_point_cloud(pcl_path):
    data = np.loadtxt(pcl_path)
    return data

def draw_registration_result(source, target, transformation):
    source_tmp = copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)

    source_tmp.paint_uniform_color([1.0, 0.0, 0.0])
    target_tmp.paint_uniform_color([0.5, 0.1, 0.0])

    source_tmp.transform(transformation)

    print(source_tmp)
    o3d.visualization.draw_geometries(
        [source_tmp, target_tmp]
    )


points = load_point_cloud('pcl_visualization/pointcloud_data/pcl0.txt')

source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(points)



points = load_point_cloud('pcl_visualization/pointcloud_data/pcl1.txt')

target = o3d.geometry.PointCloud()
target.points = o3d.utility.Vector3dVector(points)


threshold = 0.02

init_transform = np.eye(4)
draw_registration_result(source, target, init_transform)



# evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, init_transform)
# print(evaluation)

reg_icp = o3d.pipelines.registration.registration_icp(
    source, 
    target, 
    threshold, 
    init_transform, 
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 20000)
)
print(reg_icp)
print(reg_icp.transformation)
draw_registration_result(source, target, reg_icp.transformation)

