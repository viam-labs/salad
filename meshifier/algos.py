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
# target_triangles, if > 0, decimates the mesh down to roughly that many
# triangles via quadric error metrics. Used to keep the mesh cheap to use as
# a motion-planning collision obstacle.
def create_poisson_mesh(pcd, depth=9, density_quantile=0.02, target_triangles=0):
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    if density_quantile > 0:
        densities = np.asarray(densities)
        thresh = np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(densities < thresh)

    if len(mesh.vertices) == 0:
        raise RuntimeError("Poisson reconstruction produced an empty mesh")

    if target_triangles > 0 and len(mesh.triangles) > target_triangles:
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()
        mesh.remove_non_manifold_edges()
        mesh.remove_unreferenced_vertices()
        before = len(mesh.triangles)
        mesh = mesh.simplify_quadric_decimation(target_triangles)
        print(f"meshifier: decimated {before} -> {len(mesh.triangles)} "
              f"triangles (target {target_triangles})", flush=True)

    mesh.compute_vertex_normals()
    return mesh
