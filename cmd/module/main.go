package main

import (
	"salad"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
	vision "go.viam.com/rdk/services/vision"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: vision.API, Model: salad.PassthroughToCamera},
		resource.APIModel{API: vision.API, Model: salad.FileVision},
		resource.APIModel{API: genericservice.API, Model: salad.GrabberControls},
		resource.APIModel{API: genericservice.API, Model: salad.BuildCoordinator},
		resource.APIModel{API: genericservice.API, Model: salad.BowlControls},
		resource.APIModel{API: genericservice.API, Model: salad.DressingControls},
		resource.APIModel{API: genericservice.API, Model: salad.ChefsKissControls},
		resource.APIModel{API: genericservice.API, Model: salad.SupplyDetector},
	)
}