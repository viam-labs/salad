import numpy as np
import open3d as o3d


# get_point_cloud returns a pointcloud from a .pcd file found in file_name_path.
# max_nn and orient_nn are also passed in to estimate normals of the points and then
# to orient them properly.
def get_point_cloud(file_name_path, max_nn, orient_nn):
    # Load the point cloud from the .pcd file
    point_cloud = o3d.io.read_point_cloud(file_name_path)

    # Extract points as a NumPy array
    points = np.asarray(point_cloud.points)

    # Convert to Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    # Estimate normals since they are needed for meshing algorithms
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=max_nn))

    # Orient uniformly to prevent holes and normal inconsistencies
    pcd.orient_normals_consistent_tangent_plane(k=orient_nn)

    if not pcd.has_normals():
        raise ValueError("Normals could not be computed for the point cloud.")

    return pcd


# create_poisson_mesh uses Poisson surface reconstruction to create a smooth,
# hole-free mesh from a point cloud with normals.
# depth controls the level of detail: higher = finer surface (8-11 typical).
# density_quantile removes low-confidence exterior "spray" vertices; 0 keeps all.
def create_poisson_mesh(pcd, depth=9, density_quantile=0.02):
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    # Remove very low-density vertices (artefacts far from real surface).
    if density_quantile > 0:
        densities = np.asarray(densities)
        thresh = np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(densities < thresh)

    mesh.compute_vertex_normals()
    if len(mesh.vertices) == 0:
        raise RuntimeError("Poisson reconstruction produced an empty mesh")
    return mesh
