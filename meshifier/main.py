#!/usr/bin/env python3

import argparse
from algos import create_poisson_mesh, get_point_cloud
import open3d as o3d


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd_path', type=str)
    parser.add_argument('mesh_path', type=str)
    parser.add_argument('max_nn', type=str)
    parser.add_argument('orient_nn', type=str)
    # lod_mult is now the Poisson depth (8–11). 0 → default of 9.
    parser.add_argument('lod_mult', type=str)
    args = parser.parse_args()

    depth = int(args.lod_mult)
    if depth <= 0:
        depth = 9

    pcd = get_point_cloud(args.pcd_path, int(args.max_nn), int(args.orient_nn))
    mesh = create_poisson_mesh(pcd, depth=depth)
    if not o3d.io.write_triangle_mesh(args.mesh_path, mesh, write_ascii=True):
        raise Exception("failed to write mesh")


if __name__ == "__main__":
    main()
