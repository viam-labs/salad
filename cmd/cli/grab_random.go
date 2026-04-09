package main

import (
	"context"
	"fmt"
	"math"

	"github.com/spf13/cobra"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"

	"salad/segmentation"
)

type GrabRandomFlags struct {
	ZonesPath        string
	MeshPath         string
	ZoneID           int
	ArmName          string
	EndEffectorFrame string
	GripperName      string
	ScaleName        string
	HoverDistanceMM   float64
	DepthIncrementMM  float64
	MaxAttemptsPerPos int
	WeightThresholdG  float64
	ScaleDropX        float64
	ScaleDropY        float64
	ScaleDropZ        float64
	ScaleDropHoverMM  float64
}

var grabRandomFlags GrabRandomFlags

var grabRandomCmd = &cobra.Command{
	Use:   "grab-random",
	Short: "Grab from the center of a bin zone (no camera required)",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runGrabRandom(globalAddress, globalAPIKey, globalAPIKeyID, grabRandomFlags)
	},
}

func runGrabRandom(address, apiKey, apiKeyID string, flags GrabRandomFlags) error {
	if address == "" || apiKey == "" || apiKeyID == "" {
		return fmt.Errorf("--address, --api-key, and --api-key-id are required")
	}

	ctx := context.Background()
	logger := logging.NewLogger("grab-random")

	logger.Infof("Loading zones from %s", flags.ZonesPath)
	zones, err := loadZonesFile(flags.ZonesPath)
	if err != nil {
		return fmt.Errorf("loading zones: %w", err)
	}
	zone, err := findZone(zones, flags.ZoneID)
	if err != nil {
		return err
	}
	logger.Infof("Zone %d: X[%.0f,%.0f] Y[%.0f,%.0f]", zone.ID, zone.MinX, zone.MaxX, zone.MinY, zone.MaxY)

	logger.Infof("Loading mesh from %s", flags.MeshPath)
	mesh, err := spatialmath.NewMeshFromPLYFile(flags.MeshPath)
	if err != nil {
		return fmt.Errorf("loading mesh: %w", err)
	}
	worldState, err := buildWorldState(mesh)
	if err != nil {
		return fmt.Errorf("building world state: %w", err)
	}

	logger.Infof("Connecting to %s", address)
	robotClient, err := client.New(ctx, address, logger,
		client.WithDialOptions(
			rpc.WithEntityCredentials(apiKeyID, rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			}),
		),
	)
	if err != nil {
		return fmt.Errorf("connecting to robot: %w", err)
	}
	defer robotClient.Close(ctx)
	logger.Infof("Connected")

	armComp, err := arm.FromRobot(robotClient, flags.ArmName)
	if err != nil {
		names := robotClient.ResourceNames()
		logger.Infof("Available resources (%d):", len(names))
		for _, n := range names {
			logger.Infof("  %s", n)
		}
		return fmt.Errorf("arm %q: %w", flags.ArmName, err)
	}

	gripperComp, err := gripper.FromRobot(robotClient, flags.GripperName)
	if err != nil {
		return fmt.Errorf("gripper %q: %w", flags.GripperName, err)
	}

	scaleSensor, err := sensor.FromRobot(robotClient, flags.ScaleName)
	if err != nil {
		return fmt.Errorf("scale %q: %w", flags.ScaleName, err)
	}

	fsCfg, err := robotClient.FrameSystemConfig(ctx)
	if err != nil {
		return fmt.Errorf("getting frame system config: %w", err)
	}
	fs, err := referenceframe.NewFrameSystem("", fsCfg.Parts, nil)
	if err != nil {
		return fmt.Errorf("building frame system: %w", err)
	}
	fsGrab, err := buildGrabFrameSystem(fsCfg.Parts)
	if err != nil {
		return fmt.Errorf("building grab frame system: %w", err)
	}

	endEffectorFrame := flags.EndEffectorFrame
	if endEffectorFrame == "" {
		endEffectorFrame = flags.ArmName
	}

	floorZ, rimZ, target := binCenterTarget(zone, mesh)
	logger.Infof("Target: center (%.0f, %.0f) rimZ=%.0f floorZ=%.0f hoverZ=%.0f",
		target.x, target.y, rimZ, floorZ, target.z+flags.HoverDistanceMM)

	gf := GrabFlags{
		ArmName:           flags.ArmName,
		EndEffectorFrame:  flags.EndEffectorFrame,
		GripperName:       flags.GripperName,
		ScaleName:         flags.ScaleName,
		HoverDistanceMM:   flags.HoverDistanceMM,
		DepthIncrementMM:  flags.DepthIncrementMM,
		MaxAttemptsPerPos: flags.MaxAttemptsPerPos,
		WeightThresholdG:  flags.WeightThresholdG,
		ScaleDropX:        flags.ScaleDropX,
		ScaleDropY:        flags.ScaleDropY,
		ScaleDropZ:        flags.ScaleDropZ,
		ScaleDropHoverMM:  flags.ScaleDropHoverMM,
	}
	return executeGrabLoop(ctx, logger, []grabTarget{target}, armComp, gripperComp, scaleSensor, fs, fsGrab, worldState, endEffectorFrame, gf)
}

// binCenterTarget returns the bin floor Z, rim Z, and a grab target at the center XY of the
// zone with Z set to the rim (top of the bin walls). Depth-stepping descends from the rim
// toward the floor on each failed attempt. The hover (target.z + HoverDistanceMM) is above
// the rim so the approach path never intersects the bin walls.
//
// floorZ comes from the zone mesh (scanned floor region).
// rimZ comes from the full fridge mesh: max Z of any vertex within the zone XY bounds,
// which captures the actual bin wall height rather than just the floor region.
func binCenterTarget(zone *segmentation.Zone, fridgeMesh *spatialmath.Mesh) (floorZ, rimZ float64, target grabTarget) {
	centerX := (zone.MinX + zone.MaxX) / 2
	centerY := (zone.MinY + zone.MaxY) / 2

	// Floor: min Z from zone mesh (scanned floor triangles).
	floorZ = math.MaxFloat64
	for _, v := range zone.Mesh.Vertices {
		if v[2] < floorZ {
			floorZ = v[2]
		}
	}
	if floorZ == math.MaxFloat64 {
		floorZ = 0
	}

	// Rim: max Z from the full fridge mesh near the zone XY bounds.
	// We expand the search by wallMarginMM on each side so that wall vertices sitting
	// at or just outside the zone boundary are included. The interior-only search misses
	// wall tops because the segmentation zone bounds end just inside the bin walls.
	const wallMarginMM = 50.0
	rimZ = -math.MaxFloat64
	for _, tri := range fridgeMesh.Triangles() {
		for _, v := range tri.Points() {
			if v.X >= zone.MinX-wallMarginMM && v.X <= zone.MaxX+wallMarginMM &&
				v.Y >= zone.MinY-wallMarginMM && v.Y <= zone.MaxY+wallMarginMM {
				if v.Z > rimZ {
					rimZ = v.Z
				}
			}
		}
	}
	if rimZ == -math.MaxFloat64 {
		rimZ = floorZ
	}

	return floorZ, rimZ, grabTarget{centerX, centerY, rimZ}
}
