package salad

import (
	"context"
	"fmt"
	"image"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	vision "go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	vis "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	objdet "go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/viscapture"
)

var (
	FileVision = resource.NewModel("ncs", "salad", "file-vision")
)

func init() {
	resource.RegisterService(vision.API, FileVision,
		resource.Registration[vision.Service, *FileVisionConfig]{
			Constructor: newFileVision,
		},
	)
}

type FileVisionConfig struct {
	File string `json:"file"`
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
func (cfg *FileVisionConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

type fileVision struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *FileVisionConfig
	mesh   *spatialmath.Mesh

	cancelCtx  context.Context
	cancelFunc func()
	cam        camera.Camera
}

func newFileVision(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (vision.Service, error) {
	conf, err := resource.NativeConfig[*FileVisionConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewFileVision(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewFileVision(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *FileVisionConfig, logger logging.Logger) (vision.Service, error) {
	mesh, err := spatialmath.NewMeshFromPLYFile(conf.File)
	if err != nil {
		return nil, err
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &fileVision{
		mesh:       mesh,
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *fileVision) Name() resource.Name {
	return s.name
}

// DetectionsFromCamera returns a list of detections from the next image from a specified camera using a configured detector.
func (s *fileVision) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, fmt.Errorf("not implemented")
}

// Detections returns a list of detections from a given image using a configured detector.
func (s *fileVision) Detections(ctx context.Context, img image.Image, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, fmt.Errorf("not implemented")
}

// ClassificationsFromCamera returns a list of classifications from the next image from a specified camera using a configured classifier.
func (s *fileVision) ClassificationsFromCamera(ctx context.Context, cameraName string, n int, extra map[string]interface{}) (classification.Classifications, error) {
	var classificationsRetVal classification.Classifications

	return classificationsRetVal, fmt.Errorf("not implemented")
}

// Classifications returns a list of classifications from a given image using a configured classifier.
func (s *fileVision) Classifications(ctx context.Context, img image.Image, n int, extra map[string]interface{}) (classification.Classifications, error) {
	var classificationsRetVal classification.Classifications

	return classificationsRetVal, fmt.Errorf("not implemented")
}

// GetObjectPointClouds returns a list of 3D point cloud objects and metadata from the latest 3D camera image using a specified segmenter.
func (s *fileVision) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vis.Object, error) {
	pts := s.mesh.ToPoints(1)
	pc := pointcloud.NewBasicPointCloud(len(pts))
	for _, p := range pts {
		if err := pc.Set(p, pointcloud.NewBasicData()); err != nil {
			return nil, err
		}
	}
	obj, err := vis.NewObject(pc)
	if err != nil {
		return nil, err
	}
	obj.Geometry = s.mesh
	return []*vis.Object{obj}, nil
}

// properties
func (s *fileVision) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
	return nil, fmt.Errorf("not implemented")
}

// CaptureAllFromCamera returns the next image, detections, classifications, and objects all together, given a camera name. Used for
// visualization.
func (s *fileVision) CaptureAllFromCamera(ctx context.Context, cameraName string, captureOptions viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
	var visCaptureRetVal viscapture.VisCapture

	return visCaptureRetVal, fmt.Errorf("not implemented")
}

func (s *fileVision) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *fileVision) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
