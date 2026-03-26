package salad

import (
	"bytes"
	"context"
	"fmt"
	"image"
	"image/color"
	_ "image/jpeg"
	_ "image/png"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

var SupplyDetector = resource.NewModel("ncs", "salad", "supply-detector")

func init() {
	resource.RegisterService(genericservice.API, SupplyDetector,
		resource.Registration[resource.Resource, *SupplyDetectorConfig]{
			Constructor: newSupplyDetector,
		},
	)
}

type SupplyBinConfig struct {
	Name string `json:"name"`
	X1   int    `json:"x1"`
	Y1   int    `json:"y1"`
	X2   int    `json:"x2"`
	Y2   int    `json:"y2"`
}

type SupplyDetectorConfig struct {
	Camera       string            `json:"camera"`
	Bins         []SupplyBinConfig `json:"bins"`
	LowThreshold float64           `json:"low_threshold"` // fraction of non-steel pixels; below this = low
}

func (cfg *SupplyDetectorConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
	if len(cfg.Bins) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bins")
	}
	for i, b := range cfg.Bins {
		if b.Name == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'name' is required", path, i)
		}
		if b.X2 <= b.X1 || b.Y2 <= b.Y1 {
			return nil, nil, fmt.Errorf("%s.bins[%d]: invalid ROI coordinates", path, i)
		}
	}
	if cfg.LowThreshold == 0 {
		cfg.LowThreshold = 0.20
	}
	return []string{cfg.Camera}, nil, nil
}

type supplyDetector struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *SupplyDetectorConfig

	cancelCtx  context.Context
	cancelFunc func()
	cam        camera.Camera
}

func newSupplyDetector(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*SupplyDetectorConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewSupplyDetector(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewSupplyDetector(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *SupplyDetectorConfig, logger logging.Logger) (resource.Resource, error) {
	cam, err := camera.FromProvider(deps, conf.Camera)
	if err != nil {
		return nil, fmt.Errorf("failed to get camera '%s': %w", conf.Camera, err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &supplyDetector{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		cam:        cam,
	}

	s.logger.Infof("Supply detector initialized with %d bins", len(conf.Bins))
	return s, nil
}

func (s *supplyDetector) Name() resource.Name {
	return s.name
}

func (s *supplyDetector) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["check_supply"]; ok {
		return s.checkSupply(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'check_supply'")
}

func (s *supplyDetector) checkSupply(ctx context.Context) (map[string]interface{}, error) {
	imgBytes, _, err := s.cam.Image(ctx, "image/jpeg", nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get image: %w", err)
	}

	img, _, err := image.Decode(bytes.NewReader(imgBytes))
	if err != nil {
		return nil, fmt.Errorf("failed to decode image: %w", err)
	}

	result := make(map[string]interface{})

	for _, bin := range s.cfg.Bins {
		ratio, err := s.nonSteelRatio(img, bin)
		if err != nil {
			s.logger.Warnf("Failed to analyze bin '%s': %v", bin.Name, err)
			result[bin.Name] = "unknown"
			continue
		}

		s.logger.Debugf("Bin '%s': non-steel ratio = %.2f (threshold: %.2f)", bin.Name, ratio, s.cfg.LowThreshold)

		if ratio < s.cfg.LowThreshold {
			result[bin.Name] = "low"
		} else {
			result[bin.Name] = "available"
		}
	}

	return result, nil
}

// nonSteelRatio returns the fraction of pixels in the ROI that are NOT steel-colored.
// Steel is characterized by high brightness and low saturation (gray/silver).
func (s *supplyDetector) nonSteelRatio(img image.Image, bin SupplyBinConfig) (float64, error) {
	bounds := img.Bounds()

	x1 := clamp(bin.X1, bounds.Min.X, bounds.Max.X)
	y1 := clamp(bin.Y1, bounds.Min.Y, bounds.Max.Y)
	x2 := clamp(bin.X2, bounds.Min.X, bounds.Max.X)
	y2 := clamp(bin.Y2, bounds.Min.Y, bounds.Max.Y)

	if x1 >= x2 || y1 >= y2 {
		return 0, fmt.Errorf("ROI out of image bounds")
	}

	total := 0
	nonSteel := 0

	for y := y1; y < y2; y++ {
		for x := x1; x < x2; x++ {
			total++
			if !isSteelColor(img.At(x, y)) {
				nonSteel++
			}
		}
	}

	if total == 0 {
		return 0, fmt.Errorf("empty ROI")
	}

	return float64(nonSteel) / float64(total), nil
}

// isSteelColor returns true if the pixel looks like the steel bin surface.
// Steel is high brightness (>150) and low saturation (r, g, b values close together).
func isSteelColor(c color.Color) bool {
	r, g, b, _ := c.RGBA()
	// Convert from 16-bit to 8-bit
	r8 := uint8(r >> 8)
	g8 := uint8(g >> 8)
	b8 := uint8(b >> 8)

	brightness := (uint16(r8) + uint16(g8) + uint16(b8)) / 3

	maxC := max8(r8, max8(g8, b8))
	minC := min8(r8, min8(g8, b8))
	saturation := maxC - minC

	return brightness > 150 && saturation < 30
}

func clamp(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func max8(a, b uint8) uint8 {
	if a > b {
		return a
	}
	return b
}

func min8(a, b uint8) uint8 {
	if a < b {
		return a
	}
	return b
}

func (s *supplyDetector) Close(context.Context) error {
	s.cancelFunc()
	return nil
}