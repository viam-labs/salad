// salad-cli: command-line tools for the salad robot.
package main

import (
	"fmt"
	"os"
	"time"

	"github.com/spf13/cobra"

	"salad/segmentation"
)

type ScanFlags struct {
	CameraName   string
	OutputDir    string
	SleepSeconds float64
	ZOffsetMM    float64
	YMaxOffset   float64
}

func (f ScanFlags) SleepDuration() time.Duration {
	s := f.SleepSeconds
	if s <= 0 {
		s = 2
	}
	return time.Duration(s * float64(time.Second))
}

var imagingPositions = []string{
	"bin-0-imaging",
	"bin-1-imaging",
	"bin-2-imaging",
	"bin-3-imaging",
	"bin-4-imaging",
	"bin-5-imaging",
	"bin-6-imaging",
}

type DisplayFlags struct {
	LocalFiles string
	Zones      bool
	VizURL     string
	ClearFirst bool
	ShowAll    bool
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

// TODO: meshify and segment should be moved to a Viam resource/module accessible
// via DoCommand once mesh-based segmentation has been validated as an effective
// input to automated bin grabbing. Running both from the CLI is
// sufficient for initial setup purposes in the meantime.

var (
	// Persistent flags available to all subcommands.
	globalAddress  string
	globalAPIKey   string
	globalAPIKeyID string

	scanFlags    ScanFlags
	displayFlags DisplayFlags
	filterFlags  FilterFlags
	meshifyFlags MeshifyFlags
	cropFlags    CropFlags
)

var rootCmd = &cobra.Command{
	Use:   "salad-cli",
	Short: "CLI tools for the salad robot",
}

var scanCmd = &cobra.Command{
	Use:   "image",
	Short: "Capture a merged point cloud of the work area",
	Long: `Drives the left arm through each imaging position, captures a point cloud at each,
transforms each cloud into world frame via the robot's frame system, and merges them.
Per-position PCDs and the merged result are written to the output directory for inspection.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		return runScan(globalAddress, globalAPIKey, globalAPIKeyID, scanFlags)
	},
}

var displayCmd = &cobra.Command{
	Use:   "display",
	Short: "Display point clouds, meshes, and/or segment zones in motion-tools visualizer",
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

var framesCmd = &cobra.Command{
	Use:   "frames",
	Short: "Print the robot's frame system (shows all frames, parents, poses, and geometry)",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runFrames(globalAddress, globalAPIKey, globalAPIKeyID)
	},
}

func init() {
	rootCmd.PersistentFlags().StringVar(&globalAddress, "address", "", "robot address (required for scan)")
	rootCmd.PersistentFlags().StringVar(&globalAPIKey, "api-key", os.Getenv("VIAM_API_KEY"), "API key (or set VIAM_API_KEY env var)")
	rootCmd.PersistentFlags().StringVar(&globalAPIKeyID, "api-key-id", os.Getenv("VIAM_API_KEY_ID"), "API key ID (or set VIAM_API_KEY_ID env var)")

	scanCmd.Flags().StringVar(&scanFlags.CameraName, "camera", "left-downsample-cam", "camera name in robot config")
	scanCmd.Flags().StringVar(&scanFlags.OutputDir, "output", "", "output directory (default: output/<timestamp>)")
	scanCmd.Flags().Float64Var(&scanFlags.SleepSeconds, "sleep", 2.0, "seconds to wait after each arm move")
	scanCmd.Flags().Float64Var(&scanFlags.ZOffsetMM, "z-offset", -200.0, "Z offset in mm applied to every tile (negative = lower/closer to bins)")
	scanCmd.Flags().Float64Var(&scanFlags.YMaxOffset, "y-max-offset", 750.0, "upper Y offset in mm from each anchor (increase to reach further end of bins)")
	displayCmd.Flags().StringVar(&displayFlags.LocalFiles, "local-files", "output", "directory containing assets to display (.pcd, .ply, .stl; use with --zones for zones.json in this folder)")
	displayCmd.Flags().BoolVar(&displayFlags.Zones, "zones", false, "load zones.json from --local-files and draw segmented zones in the visualizer")
	displayCmd.Flags().StringVar(&displayFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	displayCmd.Flags().BoolVar(&displayFlags.ClearFirst, "clear-first", true, "clear visualizer objects before drawing")
	displayCmd.Flags().BoolVar(&displayFlags.ShowAll, "all", false, "display both point clouds and meshes (default if neither --pcd nor --mesh is set)")
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

	grabCmd.Flags().StringVar(&grabFlags.ZonesPath, "zones", "output/new_best/zones.json", "path to zones.json from segment command")
	grabCmd.Flags().StringVar(&grabFlags.MeshPath, "mesh", "output/new_best/mesh.ply", "path to bin mesh PLY file")
	grabCmd.Flags().IntVar(&grabFlags.ZoneID, "zone-id", 0, "ID of the zone to grab from")
	grabCmd.Flags().StringVar(&grabFlags.ArmName, "arm", "left-arm", "arm component name")
	grabCmd.Flags().StringVar(&grabFlags.EndEffectorFrame, "end-effector", "", "end-effector frame name for motion planning (defaults to arm name)")
	grabCmd.Flags().StringVar(&grabFlags.GripperName, "gripper", "left-gripper", "gripper component name")
	grabCmd.Flags().StringVar(&grabFlags.CameraName, "camera", "left-downsample-cam", "camera component name")
	grabCmd.Flags().StringVar(&grabFlags.ScaleName, "scale", "scale", "scale sensor component name")
	grabCmd.Flags().Float64Var(&grabFlags.SeparationMM, "separation-mm", 80.0, "minimum XY distance (mm) between grab targets")
	grabCmd.Flags().Float64Var(&grabFlags.HoverDistanceMM, "hover-mm", 100.0, "hover height (mm) above target Z for approach and retreat")
	grabCmd.Flags().Float64Var(&grabFlags.DepthIncrementMM, "depth-increment-mm", 15.0, "how much deeper (mm) to go on each failed attempt")
	grabCmd.Flags().IntVar(&grabFlags.MaxAttemptsPerPos, "max-attempts", 3, "max grab attempts per target position before moving on")
	grabCmd.Flags().Float64Var(&grabFlags.WeightThresholdG, "weight-threshold-g", 10.0, "minimum weight gain (g) to consider a grab successful")
	grabCmd.Flags().Float64Var(&grabFlags.ScaleDropX, "scale-drop-x", 320.0, "X position (mm) of scale surface in world frame")
	grabCmd.Flags().Float64Var(&grabFlags.ScaleDropY, "scale-drop-y", 456.0, "Y position (mm) of scale surface in world frame")
	grabCmd.Flags().Float64Var(&grabFlags.ScaleDropZ, "scale-drop-z", 0.0, "Z position (mm) of scale surface in world frame")
	grabCmd.Flags().Float64Var(&grabFlags.ScaleDropHoverMM, "scale-drop-hover-mm", 400.0, "hover height (mm) above scale surface before opening gripper")

	grabRandomCmd.Flags().StringVar(&grabRandomFlags.ZonesPath, "zones", "output/new_best/zones.json", "path to zones.json from segment command")
	grabRandomCmd.Flags().StringVar(&grabRandomFlags.MeshPath, "mesh", "output/new_best/mesh.ply", "path to bin mesh PLY file")
	grabRandomCmd.Flags().IntVar(&grabRandomFlags.ZoneID, "zone-id", 0, "ID of the zone to grab from")
	grabRandomCmd.Flags().StringVar(&grabRandomFlags.ArmName, "arm", "left-arm", "arm component name")
	grabRandomCmd.Flags().StringVar(&grabRandomFlags.EndEffectorFrame, "end-effector", "", "end-effector frame name for motion planning (defaults to arm name)")
	grabRandomCmd.Flags().StringVar(&grabRandomFlags.GripperName, "gripper", "left-gripper", "gripper component name")
	grabRandomCmd.Flags().StringVar(&grabRandomFlags.ScaleName, "scale", "scale", "scale sensor component name")
	grabRandomCmd.Flags().Float64Var(&grabRandomFlags.HoverDistanceMM, "hover-mm", 50.0, "hover height (mm) above the bin rim for approach and retreat")
	grabRandomCmd.Flags().Float64Var(&grabRandomFlags.DepthIncrementMM, "depth-increment-mm", 15.0, "how much deeper (mm) to go on each failed attempt")
	grabRandomCmd.Flags().IntVar(&grabRandomFlags.MaxAttemptsPerPos, "max-attempts", 3, "max grab attempts per target position before moving on")
	grabRandomCmd.Flags().Float64Var(&grabRandomFlags.WeightThresholdG, "weight-threshold-g", 10.0, "minimum weight gain (g) to consider a grab successful")
	grabRandomCmd.Flags().Float64Var(&grabRandomFlags.ScaleDropX, "scale-drop-x", 320.0, "X position (mm) of scale surface in world frame")
	grabRandomCmd.Flags().Float64Var(&grabRandomFlags.ScaleDropY, "scale-drop-y", 456.0, "Y position (mm) of scale surface in world frame")
	grabRandomCmd.Flags().Float64Var(&grabRandomFlags.ScaleDropZ, "scale-drop-z", 0.0, "Z position (mm) of scale surface in world frame")
	grabRandomCmd.Flags().Float64Var(&grabRandomFlags.ScaleDropHoverMM, "scale-drop-hover-mm", 250.0, "hover height (mm) above scale surface before opening gripper")

	rootCmd.AddCommand(scanCmd)
	rootCmd.AddCommand(displayCmd)
	rootCmd.AddCommand(filterCmd)
	rootCmd.AddCommand(meshifyCmd)
	rootCmd.AddCommand(cropCmd)
	rootCmd.AddCommand(framesCmd)
	rootCmd.AddCommand(segmentCmd)
	rootCmd.AddCommand(grabCmd)
	rootCmd.AddCommand(grabRandomCmd)
}

func main() {
	if err := rootCmd.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
