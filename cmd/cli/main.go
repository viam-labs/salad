// salad-cli: command-line tools for the salad robot.
package main

import (
	"fmt"
	"os"

	"github.com/spf13/cobra"

	"salad/filter"
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
	InputPath          string
	OutputPath         string
	VoxelMM            float64
	NeighborRadius     int
	MinNeighbors       int
	MinComponentVoxels int
	Viz                bool
	VizURL             string
}

type MeshifyFlags struct {
	InputPath       string
	OutputPath      string
	KDTreeKNN       int
	OrientNN        int
	LODMultiplier   int
	TargetTriangles int
	Viz             bool
	VizURL          string
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

var renderPlanRequestFlags RenderPlanRequestFlags

var (
	// Persistent flags available to all subcommands. Commands that dial the
	// machine (currently plane-fit, via --camera) read these through
	// resolveDialFlags, which also honors VIAM_ADDRESS / VIAM_API_KEY /
	// VIAM_API_KEY_ID env vars.
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

var renderPlanRequestCmd = &cobra.Command{
	Use:   "render-plan-request",
	Short: "Visualize a saved plan_request.json (world state, arm start, goal poses)",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runRenderPlanRequest(renderPlanRequestFlags)
	},
}

func init() {
	rootCmd.PersistentFlags().StringVar(&globalAddress, "address", os.Getenv("VIAM_ADDRESS"), "robot address (or set VIAM_ADDRESS env var)")
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
	filterCmd.Flags().StringVar(&filterFlags.OutputPath, "output", "", "output PCD file (default: output/<timestamp>/filtered.pcd)")
	filterDefaults := filter.DefaultOptions()
	filterCmd.Flags().Float64Var(&filterFlags.VoxelMM, "voxel", filterDefaults.VoxelMM, "voxel size in mm for neighbor check")
	filterCmd.Flags().IntVar(&filterFlags.NeighborRadius, "neighbor-radius", filterDefaults.NeighborRadius, "neighborhood radius in voxels; search cube is (2r+1)^3, max neighbors = (2r+1)^3 - 1")
	filterCmd.Flags().IntVar(&filterFlags.MinNeighbors, "min-neighbors", filterDefaults.MinNeighbors, "minimum occupied neighbor voxels (within --neighbor-radius cube) to keep a point; 0 disables this pass")
	filterCmd.Flags().IntVar(&filterFlags.MinComponentVoxels, "min-component-voxels", filterDefaults.MinComponentVoxels, "drop 26-connected voxel components smaller than this; 0 disables this pass")
	filterCmd.Flags().BoolVar(&filterFlags.Viz, "viz", false, "display input, kept, and removed point clouds in the motion-tools visualizer")
	filterCmd.Flags().StringVar(&filterFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	mustMarkFlagRequired(filterCmd, "input")

	meshifyCmd.Flags().StringVar(&meshifyFlags.InputPath, "input", "", "input PCD file (required)")
	meshifyCmd.Flags().StringVar(&meshifyFlags.OutputPath, "output", "", "output PLY file (default: output/<timestamp>/mesh.ply)")
	meshifyCmd.Flags().IntVar(&meshifyFlags.KDTreeKNN, "kd-tree-knn", 30, "KNN for normal estimation")
	meshifyCmd.Flags().IntVar(&meshifyFlags.OrientNN, "orient-nn", 50, "KNN for normal orientation")
	meshifyCmd.Flags().IntVar(&meshifyFlags.LODMultiplier, "lod-multiplier", 0, "Poisson reconstruction depth (8-11, higher=finer; 0=default 9)")
	meshifyCmd.Flags().IntVar(&meshifyFlags.TargetTriangles, "target-triangles", 0, "decimate output to this many triangles via quadric error metrics; 0 disables")
	meshifyCmd.Flags().BoolVar(&meshifyFlags.Viz, "viz", false, "display the output mesh in the motion-tools visualizer")
	meshifyCmd.Flags().StringVar(&meshifyFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	mustMarkFlagRequired(meshifyCmd, "input")

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
	mustMarkFlagRequired(cropCmd, "input")

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
	segmentCmd.Flags().Float64Var(&segmentFlags.FloorBandMM, "floor-band", defaults.FloorBandMM, "vertical slab (mm) above each zone's min Z from which to sample floor candidates; must accommodate tilted bins")
	segmentCmd.Flags().Float64Var(&segmentFlags.FloorMaxTiltDeg, "floor-max-tilt", defaults.FloorMaxTiltDeg, "max angle (deg) between a triangle normal and ±ẑ for it to count as a floor candidate")
	segmentCmd.Flags().IntVar(&segmentFlags.FloorMinPoints, "floor-min-points", defaults.FloorMinPoints, "minimum candidate vertices required to run a real fit; below this, fall back to horizontal plane at MinZ")
	segmentCmd.Flags().IntVar(&segmentFlags.FloorRANSACIters, "floor-ransac-iters", defaults.FloorRANSACIters, "RANSAC iterations for the bin-floor plane fit; 0 disables RANSAC and uses single-pass PCA")
	segmentCmd.Flags().Float64Var(&segmentFlags.FloorRANSACInlierMM, "floor-ransac-inlier", defaults.FloorRANSACInlierMM, "max perpendicular distance (mm) from a candidate to a RANSAC plane to count as an inlier")

	planeFitCmd.Flags().StringVar(&planeFitFlags.CameraName, "camera", "", "camera component on the machine to call NextPointCloud on (mutually exclusive with --pcd)")
	planeFitCmd.Flags().StringVar(&planeFitFlags.PCDPath, "pcd", "", "PCD file to evaluate instead of pulling from a live camera (mutually exclusive with --camera)")
	planeFitCmd.Flags().StringVar(&planeFitFlags.SavePCD, "save-pcd", "", "if --camera is set, also save the captured cloud to this path")
	planeFitCmd.Flags().StringVar(&planeFitFlags.ZonesPath, "zones", "", "zones.json with fitted bin-floor planes (required)")
	planeFitCmd.Flags().IntVar(&planeFitFlags.ZoneID, "zone-id", -1, "restrict to a single zone ID; -1 = all zones")
	planeFitCmd.Flags().StringVar(&planeFitFlags.OutputPath, "output", "", "if set, write culled PCD(s) here (file path for a single zone, directory for multiple)")
	planeFitCmd.Flags().BoolVar(&planeFitFlags.Viz, "viz", false, "draw source cloud, culled per-zone clouds, and zone plane rectangles in the motion-tools visualizer")
	planeFitCmd.Flags().StringVar(&planeFitFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	planeFitCmd.MarkFlagsMutuallyExclusive("camera", "pcd")
	planeFitCmd.MarkFlagsOneRequired("camera", "pcd")
	mustMarkFlagRequired(planeFitCmd, "zones")

	heightMapCmd.Flags().StringVar(&heightMapFlags.CameraName, "camera", "", "camera component on the machine to call NextPointCloud on (mutually exclusive with --pcd)")
	heightMapCmd.Flags().StringVar(&heightMapFlags.PCDPath, "pcd", "", "PCD file to evaluate instead of pulling from a live camera (mutually exclusive with --camera)")
	heightMapCmd.Flags().StringVar(&heightMapFlags.SavePCD, "save-pcd", "", "if --camera is set, also save the captured cloud to this path")
	heightMapCmd.Flags().StringVar(&heightMapFlags.ZonesPath, "zones", "", "zones.json with fitted bin-floor planes (required)")
	heightMapCmd.Flags().IntVar(&heightMapFlags.ZoneID, "zone-id", -1, "restrict to a single zone ID; -1 = all zones")
	heightMapCmd.Flags().Float64Var(&heightMapFlags.ServingDepthMM, "serving-depth-mm", 30, "depth below detected food surface for expected grab position (mm)")
	heightMapCmd.MarkFlagsMutuallyExclusive("camera", "pcd")
	heightMapCmd.MarkFlagsOneRequired("camera", "pcd")
	mustMarkFlagRequired(heightMapCmd, "zones")

	vizGrabPosesCmd.Flags().StringVar(&vizGrabPosesFlags.CameraName, "camera", "", "camera component on the machine to call NextPointCloud on (mutually exclusive with --pcd)")
	vizGrabPosesCmd.Flags().StringVar(&vizGrabPosesFlags.PCDPath, "pcd", "", "PCD file to evaluate instead of pulling from a live camera (mutually exclusive with --camera)")
	vizGrabPosesCmd.Flags().StringVar(&vizGrabPosesFlags.SavePCD, "save-pcd", "", "if --camera is set, also save the captured cloud to this path")
	vizGrabPosesCmd.Flags().StringVar(&vizGrabPosesFlags.ZonesPath, "zones", "", "zones.json with fitted bin-floor planes (required)")
	vizGrabPosesCmd.Flags().IntVar(&vizGrabPosesFlags.ZoneID, "zone-id", -1, "restrict to a single zone ID; -1 = all zones")
	vizGrabPosesCmd.Flags().Float64Var(&vizGrabPosesFlags.ServingDepthMM, "serving-depth-mm", 30, "depth below detected food surface for expected grab position (mm)")
	vizGrabPosesCmd.Flags().StringVar(&vizGrabPosesFlags.GrabberControls, "grabber-controls", "", "grabber-controls service name for hover and arm grab pose calibration (required)")
	vizGrabPosesCmd.Flags().StringVar(&vizGrabPosesFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	vizGrabPosesCmd.MarkFlagsMutuallyExclusive("camera", "pcd")
	vizGrabPosesCmd.MarkFlagsOneRequired("camera", "pcd")
	mustMarkFlagRequired(vizGrabPosesCmd, "zones")
	mustMarkFlagRequired(vizGrabPosesCmd, "grabber-controls")

	renderPlanRequestCmd.Flags().StringVar(&renderPlanRequestFlags.File, "file", "", "path to a saved plan_request.json (required)")
	renderPlanRequestCmd.Flags().StringVar(&renderPlanRequestFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	mustMarkFlagRequired(renderPlanRequestCmd, "file")

	rootCmd.AddCommand(displayCmd)
	rootCmd.AddCommand(filterCmd)
	rootCmd.AddCommand(meshifyCmd)
	rootCmd.AddCommand(cropCmd)
	rootCmd.AddCommand(segmentCmd)
	rootCmd.AddCommand(planeFitCmd)
	rootCmd.AddCommand(heightMapCmd)
	rootCmd.AddCommand(vizGrabPosesCmd)
	rootCmd.AddCommand(renderPlanRequestCmd)
}

func mustMarkFlagRequired(cmd *cobra.Command, name string) {
	if err := cmd.MarkFlagRequired(name); err != nil {
		panic(fmt.Sprintf("MarkFlagRequired(%q) on %q: %v", name, cmd.Use, err))
	}
}

func main() {
	if err := rootCmd.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
