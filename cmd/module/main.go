package main

import (
	"salad"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
	vision "go.viam.com/rdk/services/vision"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{API: vision.API, Model: salad.PassthroughToCamera},
		resource.APIModel{API: vision.API, Model: salad.FileVision},
		resource.APIModel{API: genericservice.API, Model: salad.GrabberControls},
		resource.APIModel{API: genericservice.API, Model: salad.BuildCoordinator},
		resource.APIModel{API: genericservice.API, Model: salad.BowlControls},
	)
}
