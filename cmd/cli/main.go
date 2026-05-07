// salad-cli: command-line tools for the salad robot.
package main

import (
	"fmt"
	"os"

	"github.com/spf13/cobra"

	"salad/segmentation"
)

type DisplayFlags struct {
	LocalFiles string
	VizURL     string
	ClearFirst bool
	ShowAll    bool
	ShowZones  bool
	ShowPCD    bool
	ShowMesh   bool
}

type FilterFlags struct {
	InputPath    string
	OutputPath   string
	VoxelMM      float64
	MinNeighbors int
}

type MeshifyFlags struct {
	InputPath     string
	OutputPath    string
	KDTreeKNN     int
	OrientNN      int
	LODMultiplier int
	Viz           bool
	VizURL        string
}

type CropFlags struct {
	InputPath  string
	OutputPath string
	MinX, MaxX float64
	MinY, MaxY float64
	MinZ, MaxZ float64
	Viz        bool
	VizURL     string
}

var (
	// Persistent flags available to all subcommands.
	globalAddress  string
	globalAPIKey   string
	globalAPIKeyID string

	displayFlags DisplayFlags
	filterFlags  FilterFlags
	meshifyFlags MeshifyFlags
	cropFlags    CropFlags
)

var rootCmd = &cobra.Command{
	Use:   "salad-cli",
	Short: "CLI tools for the salad robot",
}

var displayCmd = &cobra.Command{
	Use:   "display",
	Short: "Display local point clouds, meshes, and/or saved bin zones in motion-tools",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runDisplay(displayFlags)
	},
}

var meshifyCmd = &cobra.Command{
	Use:   "meshify",
	Short: "Convert a PCD file to a surface mesh (PLY format)",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runMeshify(meshifyFlags)
	},
}

var filterCmd = &cobra.Command{
	Use:   "filter",
	Short: "Remove noisy isolated points from a PCD file",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runFilter(filterFlags)
	},
}

var cropCmd = &cobra.Command{
	Use:   "crop",
	Short: "Crop a PCD file to an axis-aligned bounding box",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runCrop(cropFlags)
	},
}

func init() {
	rootCmd.PersistentFlags().StringVar(&globalAddress, "address", "", "robot address (required for scan)")
	rootCmd.PersistentFlags().StringVar(&globalAPIKey, "api-key", os.Getenv("VIAM_API_KEY"), "API key (or set VIAM_API_KEY env var)")
	rootCmd.PersistentFlags().StringVar(&globalAPIKeyID, "api-key-id", os.Getenv("VIAM_API_KEY_ID"), "API key ID (or set VIAM_API_KEY_ID env var)")

	displayCmd.Flags().StringVar(&displayFlags.LocalFiles, "local-files", "output", "directory containing .pcd, .ply, .stl, and/or *zones.json files to display")
	displayCmd.Flags().StringVar(&displayFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	displayCmd.Flags().BoolVar(&displayFlags.ClearFirst, "clear-first", true, "clear visualizer objects before drawing")
	displayCmd.Flags().BoolVar(&displayFlags.ShowAll, "all", false, "display point clouds, meshes, and zone meshes from *zones.json (also default content if neither --pcd nor --mesh is set)")
	displayCmd.Flags().BoolVar(&displayFlags.ShowZones, "zones", false, "display zone meshes from the newest *zones.json under --local-files")
	displayCmd.Flags().BoolVar(&displayFlags.ShowPCD, "pcd", false, "display only point clouds (when combined with --mesh, shows both)")
	displayCmd.Flags().BoolVar(&displayFlags.ShowMesh, "mesh", false, "display only meshes (when combined with --pcd, shows both)")

	filterCmd.Flags().StringVar(&filterFlags.InputPath, "input", "", "input PCD file (required)")
	filterCmd.Flags().StringVar(&filterFlags.OutputPath, "output", "", "output PCD file (required)")
	filterCmd.Flags().Float64Var(&filterFlags.VoxelMM, "voxel", 10.0, "voxel size in mm for neighbor check")
	filterCmd.Flags().IntVar(&filterFlags.MinNeighbors, "min-neighbors", 3, "minimum occupied neighbor voxels to keep a point (1-26)")
	_ = filterCmd.MarkFlagRequired("input")
	_ = filterCmd.MarkFlagRequired("output")

	meshifyCmd.Flags().StringVar(&meshifyFlags.InputPath, "input", "", "input PCD file (required)")
	meshifyCmd.Flags().StringVar(&meshifyFlags.OutputPath, "output", "", "output PLY file (default: output/<timestamp>/mesh.ply)")
	meshifyCmd.Flags().IntVar(&meshifyFlags.KDTreeKNN, "kd-tree-knn", 30, "KNN for normal estimation")
	meshifyCmd.Flags().IntVar(&meshifyFlags.OrientNN, "orient-nn", 50, "KNN for normal orientation")
	meshifyCmd.Flags().IntVar(&meshifyFlags.LODMultiplier, "lod-multiplier", 0, "Poisson reconstruction depth (8-11, higher=finer; 0=default 9)")
	meshifyCmd.Flags().BoolVar(&meshifyFlags.Viz, "viz", false, "display the output mesh in the motion-tools visualizer")
	meshifyCmd.Flags().StringVar(&meshifyFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	_ = meshifyCmd.MarkFlagRequired("input")

	cropCmd.Flags().StringVar(&cropFlags.InputPath, "input", "", "input PCD file (required)")
	cropCmd.Flags().StringVar(&cropFlags.OutputPath, "output", "", "output PCD file (default: output/<timestamp>/cropped.pcd)")
	cropCmd.Flags().Float64Var(&cropFlags.MinX, "min-x", defaultCropMinX, "minimum X to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MaxX, "max-x", defaultCropMaxX, "maximum X to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MinY, "min-y", defaultCropMinY, "minimum Y to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MaxY, "max-y", defaultCropMaxY, "maximum Y to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MinZ, "min-z", defaultCropMinZ, "minimum Z to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MaxZ, "max-z", defaultCropMaxZ, "maximum Z to keep (mm)")
	cropCmd.Flags().BoolVar(&cropFlags.Viz, "viz", false, "display the cropped point cloud in the motion-tools visualizer")
	cropCmd.Flags().StringVar(&cropFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	_ = cropCmd.MarkFlagRequired("input")

	defaults := segmentation.DefaultOptions()
	segmentCmd.Flags().StringVar(&segmentFlags.MeshPath, "mesh", "mesh.ply", "path to the fridge PLY mesh file")
	segmentCmd.Flags().StringVar(&segmentFlags.OutputDir, "output", "", "output directory (default: output/<timestamp>)")
	segmentCmd.Flags().BoolVar(&segmentFlags.Viz, "viz", false, "display each zone in a distinct color in the motion-tools visualizer")
	segmentCmd.Flags().StringVar(&segmentFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	segmentCmd.Flags().Float64Var(&segmentFlags.CellSizeMM, "cell-size", defaults.CellSizeMM, "height-map grid cell size in mm (smaller = finer boundaries)")
	segmentCmd.Flags().Float64Var(&segmentFlags.DividerZPercentile, "divider-z-percentile", defaults.DividerZPercentile, "Z percentile: cells above this are walls/dividers (absolute criterion); [0,1]")
	segmentCmd.Flags().Float64Var(&segmentFlags.DividerGradientMM, "divider-gradient", defaults.DividerGradientMM, "min Z rise (mm) above lowest neighbour to treat a cell as a divider ridge (0 = disable)")
	segmentCmd.Flags().IntVar(&segmentFlags.DividerDilation, "divider-dilation", defaults.DividerDilation, "cells to dilate barrier mask by (closes reconstruction gaps)")
	segmentCmd.Flags().Float64Var(&segmentFlags.MinZoneAreaMM2, "min-zone-area", defaults.MinZoneAreaMM2, "minimum zone footprint area in mm² (smaller components discarded as noise)")
	segmentCmd.Flags().Float64Var(&segmentFlags.MaxZoneAreaMM2, "max-zone-area", defaults.MaxZoneAreaMM2, "maximum zone footprint area in mm² (larger components rejected as non-bin regions; 0=disabled)")

	rootCmd.AddCommand(displayCmd)
	rootCmd.AddCommand(filterCmd)
	rootCmd.AddCommand(meshifyCmd)
	rootCmd.AddCommand(cropCmd)
	rootCmd.AddCommand(segmentCmd)
}

func main() {
	if err := rootCmd.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
