package salad

import (
	"context"
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
)

const (
	GrabAboveMeshFloorMM = 260.0
	ZeroChangeTolerance  = 0.5 // grams

	gripperIsMovingPoll       = 25 * time.Millisecond
	gripperGrabStopWait       = 15 * time.Second
	gripperSettleAfterStill   = 350 * time.Millisecond
	gripperSettleIfNoIsMoving = 800 * time.Millisecond
)

// WaitGripperSettledAfterGrab blocks until the gripper reports not moving, then waits
// an additional settle delay. Many gripper drivers return from Grab before the fingers
// have physically closed.
func WaitGripperSettledAfterGrab(ctx context.Context, g gripper.Gripper, logger logging.Logger) error {
	deadline := time.Now().Add(gripperGrabStopWait)
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("timeout waiting for gripper to stop after grab (%s)", gripperGrabStopWait)
		}
		moving, err := g.IsMoving(ctx)
		if err != nil {
			logger.Warnf("gripper IsMoving after grab failed (%v); using fixed settle delay", err)
			time.Sleep(gripperSettleIfNoIsMoving)
			return nil
		}
		if !moving {
			time.Sleep(gripperSettleAfterStill)
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(gripperIsMovingPoll):
		}
	}
}

// DescentWaypoints returns Z values stepping from startZ toward targetZ in increments of stepMM.
func DescentWaypoints(startZ, targetZ, stepMM float64) []float64 {
	rem := targetZ - startZ
	if rem == 0 {
		return nil
	}
	dir := 1.0
	if rem < 0 {
		dir = -1.0
	}
	var out []float64
	curZ := startZ
	for dir*(targetZ-curZ) > 0 {
		step := dir * stepMM
		if dir*step > dir*(targetZ-curZ) {
			step = targetZ - curZ
		}
		nextZ := curZ + step
		out = append(out, nextZ)
		curZ = nextZ
	}
	return out
}

// MeshBinCenterX returns the average X of rim-level vertices near (nearX, nearY).
// Used to determine lateral offset direction for retry grabs.
func MeshBinCenterX(pts []r3.Vector, nearX, nearY float64, logger logging.Logger) (float64, error) {
	if len(pts) == 0 {
		return 0, fmt.Errorf("mesh has no vertices")
	}
	maxZ, minZ := pts[0].Z, pts[0].Z
	for _, p := range pts {
		if p.Z > maxZ {
			maxZ = p.Z
		}
		if p.Z < minZ {
			minZ = p.Z
		}
	}
	rimZThresh := minZ + 0.8*(maxZ-minZ)
	const rimSearchRadius = 300.0
	var sumX float64
	var n int
	for _, p := range pts {
		if p.Z < rimZThresh {
			continue
		}
		dx, dy := p.X-nearX, p.Y-nearY
		if dx*dx+dy*dy > rimSearchRadius*rimSearchRadius {
			continue
		}
		sumX += p.X
		n++
	}
	if n < 5 {
		return 0, fmt.Errorf("only %d rim-level vertices within %.0fmm of imaging position", n, rimSearchRadius)
	}
	cx := sumX / float64(n)
	logger.Infof("Mesh rim avg X=%.0f at imaging Y=%.0f (%d rim vertices)", cx, nearY, n)
	return cx, nil
}

// MeshAvgLowestZ returns the average Z of mesh vertices near the bin floor under (cx, nearY).
func MeshAvgLowestZ(pts []r3.Vector, cx, nearY float64, logger logging.Logger) (float64, error) {
	const floorSearchRadius = 150.0
	const floorBandMM = 3.0
	var inCylinder []r3.Vector
	for _, p := range pts {
		dx, dy := p.X-cx, p.Y-nearY
		if dx*dx+dy*dy > floorSearchRadius*floorSearchRadius {
			continue
		}
		inCylinder = append(inCylinder, p)
	}
	if len(inCylinder) < 3 {
		return 0, fmt.Errorf("only %d mesh vertices within %.0fmm of (X=%.0f Y=%.0f)", len(inCylinder), floorSearchRadius, cx, nearY)
	}
	localMinZ := inCylinder[0].Z
	for _, p := range inCylinder {
		if p.Z < localMinZ {
			localMinZ = p.Z
		}
	}
	var sumZ float64
	var n int
	for _, p := range inCylinder {
		if p.Z <= localMinZ+floorBandMM {
			sumZ += p.Z
			n++
		}
	}
	if n == 0 {
		return localMinZ, nil
	}
	avgZ := sumZ / float64(n)
	logger.Infof("Mesh floor avg Z=%.0f (local min Z=%.0f, %d pts within %.0fmm of min under X=%.0f Y=%.0f)",
		avgZ, localMinZ, n, floorBandMM, cx, nearY)
	return avgZ, nil
}
