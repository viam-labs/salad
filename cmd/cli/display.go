package main

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

var meshDrawColors = []string{
	"lightblue", "lightgreen", "lightyellow", "plum", "wheat", "lightpink",
}

func runDisplay(flags DisplayFlags) error {
	files, err := resolveDisplayPaths(flags)
	if err != nil {
		return err
	}

	showPCD := flags.ShowPCD
	showMesh := flags.ShowMesh
	if flags.ShowAll {
		showPCD = true
		showMesh = true
	}
	if !showPCD && !showMesh && !flags.ShowAll {
		showPCD = true
		showMesh = true
	}

	filtered := make([]displayFile, 0, len(files))
	for _, f := range files {
		isMesh := f.kind == displayKindPLY || f.kind == displayKindSTL
		switch {
		case f.kind == displayKindPCD && showPCD:
			filtered = append(filtered, f)
		case isMesh && showMesh:
			filtered = append(filtered, f)
		}
	}
	if len(filtered) == 0 {
		return fmt.Errorf("no files to display for selected filters (pcd=%v mesh=%v)", showPCD, showMesh)
	}

	vizClient.SetURL(flags.VizURL)
	if flags.ClearFirst {
		if err := vizClient.RemoveAllSpatialObjects(); err != nil {
			return fmt.Errorf("failed to clear visualizer objects: %w", err)
		}
	}

	fmt.Printf("Drawing %d file(s) (.pcd / .ply / .stl) from %s to %s\n", len(filtered), flags.LocalFiles, flags.VizURL)
	for i, f := range filtered {
		label := displayLabel(flags.LocalFiles, f.path)
		switch f.kind {
		case displayKindPCD:
			pc, err := pointcloud.NewFromFile(f.path, "")
			if err != nil {
				return fmt.Errorf("failed to load point cloud %q: %w", f.path, err)
			}
			if err := vizClient.DrawPointCloud(label, pc, nil); err != nil {
				return fmt.Errorf("failed to draw point cloud %q: %w", f.path, err)
			}
		case displayKindPLY:
			mesh, err := spatialmath.NewMeshFromPLYFile(f.path)
			if err != nil {
				return fmt.Errorf("failed to load mesh %q: %w", f.path, err)
			}
			mesh.SetLabel(label)
			color := meshDrawColors[i%len(meshDrawColors)]
			if err := vizClient.DrawGeometry(mesh, color); err != nil {
				return fmt.Errorf("failed to draw mesh %q: %w", f.path, err)
			}
		case displayKindSTL:
			mesh, err := spatialmath.NewMeshFromSTLFile(f.path)
			if err != nil {
				return fmt.Errorf("failed to load mesh %q: %w", f.path, err)
			}
			mesh.SetLabel(label)
			color := meshDrawColors[i%len(meshDrawColors)]
			if err := vizClient.DrawGeometry(mesh, color); err != nil {
				return fmt.Errorf("failed to draw mesh %q: %w", f.path, err)
			}
		default:
			return fmt.Errorf("unsupported display kind for %q", f.path)
		}
		time.Sleep(300 * time.Millisecond)
	}

	fmt.Println("Display complete.")
	return nil
}

func displayLabel(baseDir, path string) string {
	label := strings.TrimPrefix(path, baseDir)
	label = strings.TrimPrefix(label, string(filepath.Separator))
	if label == "" {
		label = filepath.Base(path)
	}
	return label
}

type displayKind int

const (
	displayKindPCD displayKind = iota
	displayKindPLY
	displayKindSTL
)

type displayFile struct {
	path    string
	kind    displayKind
	modTime int64
}

func resolveDisplayPaths(flags DisplayFlags) ([]displayFile, error) {
	if flags.LocalFiles == "" {
		return nil, fmt.Errorf("--local-files directory is required")
	}

	info, err := os.Stat(flags.LocalFiles)
	if err != nil {
		return nil, fmt.Errorf("failed to access --local-files %q: %w", flags.LocalFiles, err)
	}
	if !info.IsDir() {
		return nil, fmt.Errorf("--local-files must be a directory, got %q", flags.LocalFiles)
	}

	var matches []displayFile
	err = filepath.WalkDir(flags.LocalFiles, func(path string, d os.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		if d.IsDir() {
			return nil
		}
		name := strings.ToLower(d.Name())
		var kind displayKind
		switch {
		case strings.HasSuffix(name, ".pcd"):
			kind = displayKindPCD
		case strings.HasSuffix(name, ".ply"):
			kind = displayKindPLY
		case strings.HasSuffix(name, ".stl"):
			kind = displayKindSTL
		default:
			return nil
		}
		info, err := d.Info()
		if err != nil {
			return err
		}
		matches = append(matches, displayFile{
			path:    path,
			kind:    kind,
			modTime: info.ModTime().UnixNano(),
		})
		return nil
	})
	if err != nil {
		return nil, fmt.Errorf("failed to scan folder %q: %w", flags.LocalFiles, err)
	}
	if len(matches) == 0 {
		return nil, fmt.Errorf("no .pcd, .ply, or .stl files found under %q", flags.LocalFiles)
	}

	sort.Slice(matches, func(i, j int) bool {
		if matches[i].modTime == matches[j].modTime {
			return matches[i].path < matches[j].path
		}
		return matches[i].modTime < matches[j].modTime
	})
	return matches, nil
}
