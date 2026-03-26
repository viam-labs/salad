#!/usr/bin/env python3

import argparse
from algos import create_BPA_mesh, get_point_cloud
import open3d as o3d


def main():
    # Create the argument parser
    parser = argparse.ArgumentParser()

    # define the command-line arguments
    parser.add_argument('pcd_path', type=str)
    parser.add_argument('mesh_path', type=str)
    parser.add_argument('max_nn', type=str)
    parser.add_argument('orient_nn', type=str)
    parser.add_argument('lod_mult', type=str)

    # parse the arguments
    args = parser.parse_args()

    pcd = get_point_cloud(args.pcd_path, int(args.max_nn), int(args.orient_nn))

    # create the actual mesh
    mesh = create_BPA_mesh(pcd, int(args.lod_mult))
    if not o3d.io.write_triangle_mesh(args.mesh_path, mesh, write_ascii=True):
        raise Exception("failed to write mesh")



if __name__ == "__main__":
    main()
