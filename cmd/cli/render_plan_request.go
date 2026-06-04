package main

import (
	"context"
	"fmt"
	"os"

	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

type RenderPlanRequestFlags struct {
	File   string
	VizURL string
}

func runRenderPlanRequest(flags RenderPlanRequestFlags) error {
	if flags.File == "" {
		return fmt.Errorf("--file is required")
	}

	req, err := armplanning.ReadRequestFromFile(flags.File)
	if err != nil {
		return fmt.Errorf("reading plan request %q: %w", flags.File, err)
	}
	if req.FrameSystem == nil {
		return fmt.Errorf("plan request has no frame system")
	}
	if req.StartState == nil {
		return fmt.Errorf("plan request has no start state")
	}

	vizClient.SetURL(flags.VizURL)
	if err := vizClient.RemoveAllSpatialObjects(); err != nil {
		return fmt.Errorf("clearing visualizer: %w", err)
	}

	startInputs := req.StartState.Configuration()
	if startInputs == nil {
		startInputs = referenceframe.NewZeroInputs(req.FrameSystem)
	}

	if req.WorldState != nil {
		if err := vizClient.DrawWorldState(req.WorldState, req.FrameSystem, startInputs); err != nil {
			return fmt.Errorf("drawing world state: %w", err)
		}
	}

	if err := vizClient.DrawFrameSystem(req.FrameSystem, startInputs); err != nil {
		return fmt.Errorf("drawing frame system: %w", err)
	}

	for i, goal := range req.Goals {
		poses := goal.Poses()
		if len(poses) == 0 {
			computed, err := goal.ComputePoses(context.Background(), req.FrameSystem)
			if err != nil {
				fmt.Fprintf(os.Stderr, "goal %d: could not compute poses: %v\n", i, err)
				continue
			}
			poses = computed
		}
		flat := make([]spatialmath.Pose, 0, len(poses))
		for _, pif := range poses {
			flat = append(flat, pif.Pose())
		}
		if err := vizClient.DrawPoses(flat, []string{"green"}, true); err != nil {
			return fmt.Errorf("drawing goal %d poses: %w", i, err)
		}
		fmt.Printf("goal %d: %d pose(s)\n", i, len(flat))
		for name, pif := range poses {
			pt := pif.Pose().Point()
			fmt.Printf("  %s: x=%.2f y=%.2f z=%.2f (parent=%s)\n", name, pt.X, pt.Y, pt.Z, pif.Parent())
		}
	}

	fmt.Println()
	fmt.Println("start state:")
	for frame, inputs := range startInputs {
		fmt.Printf("  %s: %v\n", frame, inputs)
	}

	fmt.Println()
	if req.Constraints != nil {
		fmt.Printf("constraints: %+v\n", req.Constraints)
	} else {
		fmt.Println("constraints: <none>")
	}

	return nil
}
