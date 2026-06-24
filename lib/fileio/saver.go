package fileio

import (
	"context"
	"fmt"
	"path/filepath"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"golang.org/x/sync/errgroup"
)

const timestampLayout = "January_02_2006_15_04_05"

type SaveFile struct {
	BuildID          string
	Filename         string
	Timestamp        string
	Req              *armplanning.PlanRequest
	PlanningDuration time.Duration
}

func (sf SaveFile) Filepath(baseDir string) string {
	return filepath.Join(
		baseDir,
		fmt.Sprintf("tag=%s", sf.BuildID),
		fmt.Sprintf("%s_%s", sf.Timestamp, sf.Filename),
	)
}

func NewPlanRequestSaveFile(req *armplanning.PlanRequest, buildID, filename string, now time.Time, planningDuration time.Duration) SaveFile {
	return SaveFile{
		BuildID:          buildID,
		Filename:         filename,
		Timestamp:        now.Format(timestampLayout),
		Req:              req,
		PlanningDuration: planningDuration,
	}
}

// FileSaver writes plan-request protobuf files asynchronously.
// All methods are safe to call on a nil *FileSaver (they no-op).
type FileSaver struct {
	baseDir    string
	ch         chan SaveFile
	bufferSize int
	ctx        context.Context
	cancel     context.CancelFunc
	g          *errgroup.Group
	logger     logging.Logger
}

func NewFileSaver(logger logging.Logger, baseDir string) *FileSaver {
	const bufferSize = 500
	const numWorkers = 2
	ctx, cancel := context.WithCancel(context.Background())
	s := &FileSaver{
		baseDir:    baseDir,
		logger:     logger,
		ch:         make(chan SaveFile, bufferSize),
		bufferSize: bufferSize,
		ctx:        ctx,
		cancel:     cancel,
		g:          new(errgroup.Group),
	}
	for i := 0; i < numWorkers; i++ {
		s.g.Go(func() error {
			for {
				select {
				case sf, ok := <-s.ch:
					if !ok {
						return nil
					}
					path := sf.Filepath(s.baseDir)
					if err := EnsureDir(filepath.Dir(path)); err != nil {
						s.logger.Warnw("FileSaver mkdir failed", "path", path, "err", err)
						continue
					}
					if err := sf.Req.WriteToFile(path); err != nil {
						s.logger.Warnw("FileSaver write failed", "path", path, "err", err)
						continue
					}
					s.logger.Infof("FileSaver wrote %s (planning %s, %d remaining)", path, sf.PlanningDuration, len(s.ch))
				case <-s.ctx.Done():
					return nil
				}
			}
		})
	}
	return s
}

func (s *FileSaver) SaveAsync(ctx context.Context, sf SaveFile) {
	if s == nil || sf.BuildID == "" {
		return
	}
	if ctx.Err() != nil || s.ctx.Err() != nil {
		return
	}
	if len(s.ch) > int(0.8*float64(s.bufferSize)) {
		s.logger.Warnf("FileSaver channel nearly full: %d/%d", len(s.ch), s.bufferSize)
	}
	select {
	case s.ch <- sf:
	case <-ctx.Done():
	case <-s.ctx.Done():
	}
}

// Close cancels in-flight writes after a 5s grace period for draining.
func (s *FileSaver) Close() error {
	if s == nil {
		return nil
	}
	if remaining := len(s.ch); remaining > 0 {
		s.logger.Warnf("FileSaver closing with %d files remaining; waiting up to 5s", remaining)
		timer := time.NewTimer(5 * time.Second)
		ticker := time.NewTicker(200 * time.Millisecond)
		defer timer.Stop()
		defer ticker.Stop()
	outer:
		for {
			select {
			case <-timer.C:
				break outer
			case <-ticker.C:
				if len(s.ch) == 0 {
					break outer
				}
			}
		}
	}
	s.cancel()
	return s.g.Wait()
}
