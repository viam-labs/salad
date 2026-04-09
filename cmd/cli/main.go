// salad-cli: command-line tools for the salad robot.
package main

import (
	"fmt"
	"math"
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
}

type CropFlags struct {
	InputPath  string
	OutputPath string
	MinX, MaxX float64
	MinY, MaxY float64
	MinZ, MaxZ float64
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
	Short: "Display local point clouds and meshes in motion-tools visualizer",
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
	displayCmd.Flags().StringVar(&displayFlags.LocalFiles, "local-files", "output", "directory containing .pcd, .ply, and/or .stl files to display")
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
	meshifyCmd.Flags().StringVar(&meshifyFlags.OutputPath, "output", "", "output PLY file (required)")
	meshifyCmd.Flags().IntVar(&meshifyFlags.KDTreeKNN, "kd-tree-knn", 30, "KNN for normal estimation")
	meshifyCmd.Flags().IntVar(&meshifyFlags.OrientNN, "orient-nn", 50, "KNN for normal orientation")
	meshifyCmd.Flags().IntVar(&meshifyFlags.LODMultiplier, "lod-multiplier", 0, "Poisson reconstruction depth (8-11, higher=finer; 0=default 9)")
	_ = meshifyCmd.MarkFlagRequired("input")
	_ = meshifyCmd.MarkFlagRequired("output")

	cropCmd.Flags().StringVar(&cropFlags.InputPath, "input", "", "input PCD file (required)")
	cropCmd.Flags().StringVar(&cropFlags.OutputPath, "output", "", "output PCD file (required)")
	cropCmd.Flags().Float64Var(&cropFlags.MinX, "min-x", -math.MaxFloat64, "minimum X to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MaxX, "max-x", math.MaxFloat64, "maximum X to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MinY, "min-y", -math.MaxFloat64, "minimum Y to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MaxY, "max-y", math.MaxFloat64, "maximum Y to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MinZ, "min-z", -math.MaxFloat64, "minimum Z to keep (mm)")
	cropCmd.Flags().Float64Var(&cropFlags.MaxZ, "max-z", math.MaxFloat64, "maximum Z to keep (mm)")
	_ = cropCmd.MarkFlagRequired("input")
	_ = cropCmd.MarkFlagRequired("output")

	defaults := segmentation.DefaultOptions()
	segmentCmd.Flags().StringVar(&segmentFlags.MeshPath, "mesh", "mesh.ply", "path to the fridge PLY mesh file")
	segmentCmd.Flags().StringVar(&segmentFlags.OutputPath, "output", "", "output JSON path (default: <mesh>.zones.json)")
	segmentCmd.Flags().BoolVar(&segmentFlags.Viz, "viz", false, "display each zone in a distinct color in the motion-tools visualizer")
	segmentCmd.Flags().StringVar(&segmentFlags.VizURL, "viz-url", "http://localhost:3000", "motion-tools visualizer URL")
	segmentCmd.Flags().Float64Var(&segmentFlags.CellSizeMM, "cell-size", defaults.CellSizeMM, "height-map grid cell size in mm (smaller = finer boundaries)")
	segmentCmd.Flags().Float64Var(&segmentFlags.DividerZPercentile, "divider-z-percentile", defaults.DividerZPercentile, "Z percentile threshold separating dividers (high) from bin surfaces (low); [0,1]")
	segmentCmd.Flags().IntVar(&segmentFlags.DividerDilation, "divider-dilation", defaults.DividerDilation, "cells to dilate divider mask by (closes gaps in Poisson-reconstructed dividers)")
	segmentCmd.Flags().Float64Var(&segmentFlags.MinBinAreaMM2, "min-bin-area", defaults.MinBinAreaMM2, "minimum zone footprint area in mm² (smaller components discarded as noise)")

	rootCmd.AddCommand(scanCmd)
	rootCmd.AddCommand(displayCmd)
	rootCmd.AddCommand(filterCmd)
	rootCmd.AddCommand(meshifyCmd)
	rootCmd.AddCommand(cropCmd)
	rootCmd.AddCommand(framesCmd)
	rootCmd.AddCommand(segmentCmd)
}

func main() {
	if err := rootCmd.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
