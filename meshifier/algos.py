import numpy as np
import open3d as o3d

# multiplier for avg_dist found in create_BPA_mesh
# the size of avg_dist is increased so that the rolling ball
# does not fall through any points. This value is set to three
# so that the rolling ball smooths out noisy points in the underlying pointcloud.
avg_dist_multiplier = 3

# create_BPA_mesh consumes a pointcloud and creates a mesh from it using the
# ball pivoting algorithm.
# the resulting mesh then has its area measured and is then multiplied by
# lod_multiplier to give the desired level of detail.
def create_BPA_mesh(pcd, lod_multiplier):
    # find the average distance between points
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)

    # create mesh using ball pivoting
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector([avg_dist_multiplier * avg_dist])
    )

    # area is in meters squared
    area = bpa_mesh.get_surface_area()

    # generate mesh at desired LoD
    return bpa_mesh.simplify_quadric_decimation(int(area*lod_multiplier))


# get_point_cloud returns a pointcloud from a .pcd file found in file_name_path.
# max_nn and orient_nn are also passed in to estimate normals of the points and then
# to orient them properly.
def get_point_cloud(file_name_path, max_nn, orient_nn):
    # Load the point cloud from the .pcd file
    point_cloud = o3d.io.read_point_cloud(file_name_path)

    # Extract points as a NumPy array
    points = np.asarray(point_cloud.points)

    # If needed, perform slicing or other operations on the NumPy array
    points_subset = points[:, :3]

    # Convert to Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_subset)

    # Downsample to reduce computational load
    downSampled_PointCloud = de_duplicate(pcd)

    # Estimate normals since they are needed for meshing algorithms
    downSampled_PointCloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=max_nn))

    # Orient uniformly to prevent holes and normal inconsistencies
    downSampled_PointCloud.orient_normals_consistent_tangent_plane(k=orient_nn)

    # Check if normals are assigned
    if not downSampled_PointCloud.has_normals():
        raise ValueError("Normals could not be computed for the point cloud.")

    return downSampled_PointCloud

# de_deduplicate minimizes the number of points in a pointcloud by the voxel_size
def de_duplicate(pcd):
    # voxel_down_sample operates in the same units as the pointcloud's coordinates
    return pcd.voxel_down_sample(voxel_size=0.01)
