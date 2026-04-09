package main

import (
	"fmt"
	"salad/meshification"
)

func runMeshify(flags MeshifyFlags) error {
	fmt.Printf("Meshing %s → %s\n", flags.InputPath, flags.OutputPath)
	cfg := meshification.Config{
		KDTreeKNN:     flags.KDTreeKNN,
		OrientNN:      flags.OrientNN,
		LODMultiplier: flags.LODMultiplier,
	}
	if err := meshification.Run(flags.InputPath, flags.OutputPath, cfg); err != nil {
		return err
	}
	fmt.Printf("Wrote %s\n", flags.OutputPath)
	return nil
}
