package salad

import (
	"context"
	"fmt"
	"image"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	vision "go.viam.com/rdk/services/vision"
	vis "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	objdet "go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/viscapture"
)

var (
	PassthroughToCamera = resource.NewModel("ncs", "salad", "passthrough-to-camera")
	errUnimplemented    = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(vision.API, PassthroughToCamera,
		resource.Registration[vision.Service, *Config]{
			Constructor: newSaladPassthroughToCamera,
		},
	)
}

type Config struct {
	Camera string `json:"camera"`
	/*
		Put config attributes here. There should be public/exported fields
		with a `json` parameter at the end of each attribute.

		Example config struct:
			type Config struct {
				Pin   string `json:"pin"`
				Board string `json:"board"`
				MinDeg *float64 `json:"min_angle_deg,omitempty"`
			}

		If your model does not need a config, replace *Config in the init
		function with resource.NoNativeConfig
	*/
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns three values:
//  1. Required dependencies: other resources that must exist for this resource to work.
//  2. Optional dependencies: other resources that may exist but are not required.
//  3. An error if any Config fields are missing or invalid.
//
// The `path` parameter indicates
// where this resource appears in the machine's JSON configuration
// (for example, "components.0"). You can use it in error messages
// to indicate which resource has a problem.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	return []string{cfg.Camera}, nil, nil
}

type saladPassthroughToCamera struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()
	cam        camera.Camera
}

func newSaladPassthroughToCamera(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (vision.Service, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewPassthroughToCamera(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewPassthroughToCamera(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (vision.Service, error) {
	cam, err := camera.FromProvider(deps, conf.Camera)
	if err != nil {
		return nil, err
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &saladPassthroughToCamera{
		cam:        cam,
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *saladPassthroughToCamera) Name() resource.Name {
	return s.name
}

// DetectionsFromCamera returns a list of detections from the next image from a specified camera using a configured detector.
func (s *saladPassthroughToCamera) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, fmt.Errorf("not implemented")
}

// Detections returns a list of detections from a given image using a configured detector.
func (s *saladPassthroughToCamera) Detections(ctx context.Context, img image.Image, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, fmt.Errorf("not implemented")
}

// ClassificationsFromCamera returns a list of classifications from the next image from a specified camera using a configured classifier.
func (s *saladPassthroughToCamera) ClassificationsFromCamera(ctx context.Context, cameraName string, n int, extra map[string]interface{}) (classification.Classifications, error) {
	var classificationsRetVal classification.Classifications

	return classificationsRetVal, fmt.Errorf("not implemented")
}

// Classifications returns a list of classifications from a given image using a configured classifier.
func (s *saladPassthroughToCamera) Classifications(ctx context.Context, img image.Image, n int, extra map[string]interface{}) (classification.Classifications, error) {
	var classificationsRetVal classification.Classifications

	return classificationsRetVal, fmt.Errorf("not implemented")
}

// GetObjectPointClouds returns a list of 3D point cloud objects and metadata from the latest 3D camera image using a specified segmenter.
func (s *saladPassthroughToCamera) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vis.Object, error) {
	pc, err := s.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, err
	}
	obj := vis.NewEmptyObject()
	obj.PointCloud = pc
	return []*vis.Object{obj}, nil
}

// properties
func (s *saladPassthroughToCamera) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
	return nil, fmt.Errorf("not implemented")
}

// CaptureAllFromCamera returns the next image, detections, classifications, and objects all together, given a camera name. Used for
// visualization.
func (s *saladPassthroughToCamera) CaptureAllFromCamera(ctx context.Context, cameraName string, captureOptions viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
	var visCaptureRetVal viscapture.VisCapture

	return visCaptureRetVal, fmt.Errorf("not implemented")
}

func (s *saladPassthroughToCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *saladPassthroughToCamera) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
