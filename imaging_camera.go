// Package salad provides a bin-imaging camera that gates its arm-moving capture
// sequence behind a DoCommand. NextPointCloud returns a cached merged point
// cloud and never moves the arm, so casual callers (e.g. the app's 3D scene
// tab, data capture) can poll without interfering with salad assembly.
//
// Behavior:
//   - DoCommand({"capture": true}) drives the arm through each configured
//     position switch, captures a point cloud at each, merges them into the
//     world frame, and stores the result in cache.
//   - NextPointCloud returns the cached cloud or errors if no capture has
//     happened yet.
package salad

import (
	"context"
	"fmt"
	"sync"
	"time"

	touch "github.com/erh/vmodutils/touch"

	"go.viam.com/rdk/components/camera"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

var BinImagingCamera = resource.NewModel("ncs", "salad", "bin-imaging-camera")

func init() {
	resource.RegisterComponent(
		camera.API,
		BinImagingCamera,
		resource.Registration[camera.Camera, *BinImagingCameraConfig]{
			Constructor: newBinImagingCamera,
		},
	)
}

type BinImagingCameraConfig struct {
	Src          string   `json:"src"`
	SleepSeconds float64  `json:"sleep_seconds"`
	Positions    []string `json:"positions"`
}

func (c *BinImagingCameraConfig) sleepTime() time.Duration {
	if c.SleepSeconds <= 0 {
		return time.Second
	}
	return time.Duration(c.SleepSeconds * float64(time.Second))
}

func (c *BinImagingCameraConfig) Validate(path string) ([]string, []string, error) {
	if c.Src == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "src")
	}
	if len(c.Positions) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "positions")
	}
	deps := make([]string, 0, 1+len(c.Positions))
	deps = append(deps, c.Src)
	deps = append(deps, c.Positions...)
	return deps, nil, nil
}

type binImagingCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	logger logging.Logger
	cfg    *BinImagingCameraConfig

	fsSvc     framesystem.Service
	src       camera.Camera
	positions []toggleswitch.Switch

	// captureMu serializes capture sequences so two callers can't fight over the arm.
	captureMu sync.Mutex
	cacheMu   sync.RWMutex
	cachedPC  pointcloud.PointCloud
}

func newBinImagingCamera(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (camera.Camera, error) {
	conf, err := resource.NativeConfig[*BinImagingCameraConfig](rawConf)
	if err != nil {
		return nil, err
	}

	src, err := camera.FromProvider(deps, conf.Src)
	if err != nil {
		return nil, fmt.Errorf("failed to get src camera %q: %w", conf.Src, err)
	}

	positions := make([]toggleswitch.Switch, 0, len(conf.Positions))
	for _, name := range conf.Positions {
		sw, err := toggleswitch.FromProvider(deps, name)
		if err != nil {
			return nil, fmt.Errorf("failed to get position switch %q: %w", name, err)
		}
		positions = append(positions, sw)
	}

	fsSvc, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}

	logger.Infof("bin-imaging-camera initialized with %d positions, src=%q", len(positions), conf.Src)
	return &binImagingCamera{
		name:      rawConf.ResourceName(),
		logger:    logger,
		cfg:       conf,
		fsSvc:     fsSvc,
		src:       src,
		positions: positions,
	}, nil
}

func (c *binImagingCamera) Name() resource.Name {
	return c.name
}

func (c *binImagingCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["capture"]; !ok {
		return nil, fmt.Errorf("unknown command, expected 'capture' field")
	}

	c.captureMu.Lock()
	defer c.captureMu.Unlock()

	c.logger.Infof("running bin-imaging capture across %d positions", len(c.positions))
	pc, err := touch.GetMergedPointCloudFromPositions(ctx, c.positions, c.cfg.sleepTime(), c.src, nil, c.fsSvc, false)
	if err != nil {
		return nil, fmt.Errorf("capture failed: %w", err)
	}

	c.cacheMu.Lock()
	c.cachedPC = pc
	c.cacheMu.Unlock()

	c.logger.Infof("bin-imaging capture complete: %d points cached", pc.Size())
	return map[string]interface{}{
		"success": true,
		"points":  pc.Size(),
	}, nil
}

func (c *binImagingCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	c.cacheMu.RLock()
	pc := c.cachedPC
	c.cacheMu.RUnlock()
	if pc == nil {
		return nil, fmt.Errorf("no cached point cloud, call DoCommand({\"capture\": true}) first")
	}
	return pc, nil
}

func (c *binImagingCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{SupportsPCD: true}, nil
}

func (c *binImagingCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	return nil, camera.ImageMetadata{}, fmt.Errorf("image not supported")
}

func (c *binImagingCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	return nil, resource.ResponseMetadata{}, fmt.Errorf("images not supported")
}

func (c *binImagingCamera) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}
