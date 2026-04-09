#!/usr/bin/env python3

import argparse
import os
import numpy as np
from algos import create_poisson_mesh, get_point_cloud
import open3d as o3d


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd_path', type=str)
    parser.add_argument('mesh_path', type=str)
    parser.add_argument('max_nn', type=str)
    parser.add_argument('orient_nn', type=str)
    parser.add_argument('lod_mult', type=str)
    parser.add_argument('voxel_size', type=str, nargs='?', default='0.0')
    parser.add_argument('smooth_iterations', type=str, nargs='?', default='0')
    args = parser.parse_args()

    depth = int(args.lod_mult)
    if depth <= 0:
        depth = 9

    smooth_iterations = int(args.smooth_iterations)
    if smooth_iterations <= 0:
        smooth_iterations = 15

    mesh_path = os.path.abspath(args.mesh_path)

    pcd = get_point_cloud(args.pcd_path, int(args.max_nn), int(args.orient_nn), voxel_size=float(args.voxel_size))
    mesh = create_poisson_mesh(pcd, depth=depth, smooth_iterations=smooth_iterations)

    verts = np.asarray(mesh.vertices)
    if not np.isfinite(verts).all():
        raise ValueError(f"mesh contains NaN/inf vertices ({(~np.isfinite(verts)).sum()} bad values) — Poisson topology may be too degenerate to smooth")

    if not o3d.io.write_triangle_mesh(mesh_path, mesh, write_ascii=True):
        raise Exception(f"write_triangle_mesh returned False for {mesh_path}")

    if not os.path.exists(mesh_path):
        raise Exception(f"write_triangle_mesh returned True but file was not created at {mesh_path}")


if __name__ == "__main__":
    main()
