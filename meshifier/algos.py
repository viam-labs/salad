import numpy as np
import open3d as o3d


# get_point_cloud returns a pointcloud from a .pcd file found in file_name_path.
# max_nn and orient_nn are also passed in to estimate normals of the points and then
# to orient them properly.
def get_point_cloud(file_name_path, max_nn, orient_nn, voxel_size=0.0):
    point_cloud = o3d.io.read_point_cloud(file_name_path)

    points = np.asarray(point_cloud.points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)

    n_points = len(pcd.points)
    if n_points < 4:
        raise ValueError(f"point cloud has only {n_points} points after downsampling — voxel_size may be too large")

    max_nn = min(max_nn, n_points - 1)
    orient_nn = min(orient_nn, n_points - 1)

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=max_nn))

    pcd.orient_normals_consistent_tangent_plane(k=orient_nn)

    if not pcd.has_normals():
        raise ValueError("Normals could not be computed for the point cloud.")

    return pcd


# create_poisson_mesh uses Poisson surface reconstruction to create a smooth,
# hole-free mesh from a point cloud with normals.
# depth controls the level of detail: higher = finer surface (8-11 typical).
# density_quantile removes low-confidence exterior "spray" vertices; 0 keeps all.
def create_poisson_mesh(pcd, depth=9, density_quantile=0.02, smooth_iterations=15):
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    if density_quantile > 0:
        densities = np.asarray(densities)
        thresh = np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(densities < thresh)

    if len(mesh.vertices) == 0:
        raise ValueError("Poisson reconstruction produced an empty mesh — check point cloud density and normal quality")

    mesh.remove_non_manifold_edges()
    mesh = mesh.filter_smooth_taubin(number_of_iterations=smooth_iterations)

    if len(mesh.vertices) == 0:
        raise ValueError("Taubin smoothing produced an empty mesh — Poisson topology may be too degenerate to smooth")

    mesh.compute_vertex_normals()
    return mesh
