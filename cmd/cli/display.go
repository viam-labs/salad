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

	"salad/segmentation"
)

var meshDrawColors = []string{
	"lightblue", "lightgreen", "lightyellow", "plum", "wheat", "lightpink",
}

var zoneVizColors = []string{
	"red", "blue", "green", "orange", "purple",
	"cyan", "yellow", "pink", "teal", "chocolate",
}

type zoneVizOptions struct {
	ClearFirst    bool
	VizURL        string
	LocalDir      string
	PreferredMesh string
	Result        *segmentation.ZonesResult
}

// drawFullZonesViz optionally clears, draws a grey source mesh if resolvable, then each zone in a distinct color.
func drawFullZonesViz(opts zoneVizOptions) error {
	if opts.Result == nil {
		return fmt.Errorf("no zone result to draw")
	}
	vizClient.SetURL(opts.VizURL)
	if opts.ClearFirst {
		if err := vizClient.RemoveAllSpatialObjects(); err != nil {
			return fmt.Errorf("clearing visualizer: %w", err)
		}
	}
	if p, ok := resolveZoneBackgroundMeshPath(opts.LocalDir, opts.PreferredMesh, opts.Result.SourceMesh); ok {
		srcMesh, err := spatialmath.NewMeshFromPLYFile(p)
		if err != nil {
			return fmt.Errorf("loading mesh for zone viz: %w", err)
		}
		srcMesh.SetLabel("fridge-mesh")
		if err := vizClient.DrawGeometry(srcMesh, "lightgrey"); err != nil {
			return fmt.Errorf("drawing background mesh: %w", err)
		}
		time.Sleep(200 * time.Millisecond)
	}
	if err := appendZoneColoredMeshes(opts.Result); err != nil {
		return err
	}
	return nil
}

// appendZoneColoredMeshes draws zone sub-meshes only (no clear, no background).
func appendZoneColoredMeshes(result *segmentation.ZonesResult) error {
	if result == nil {
		return fmt.Errorf("no zone result to draw")
	}
	for _, zone := range result.Zones {
		label := fmt.Sprintf("zone-%d", zone.ID)
		color := zoneVizColors[zone.ID%len(zoneVizColors)]
		zoneMesh := zone.Mesh.ToSpatialMesh(label)
		if err := vizClient.DrawGeometry(zoneMesh, color); err != nil {
			return fmt.Errorf("drawing zone %d: %w", zone.ID, err)
		}
		time.Sleep(100 * time.Millisecond)
	}
	return nil
}

func resolveZoneBackgroundMeshPath(localDir, preferredMesh, sourceMeshInJSON string) (string, bool) {
	if preferredMesh != "" {
		if st, err := os.Stat(preferredMesh); err == nil && !st.IsDir() {
			return preferredMesh, true
		}
	}
	if sourceMeshInJSON == "" {
		return "", false
	}
	candidates := []string{
		filepath.Join(localDir, filepath.Base(sourceMeshInJSON)),
		sourceMeshInJSON,
	}
	for _, p := range candidates {
		if p == "" {
			continue
		}
		if st, err := os.Stat(p); err == nil && !st.IsDir() {
			return p, true
		}
	}
	return "", false
}

// findNewestZonesJSON returns the path of the most recently modified *zones.json under dir.
func findNewestZonesJSON(dir string) (string, error) {
	var best struct {
		path    string
		modTime int64
	}
	err := filepath.WalkDir(dir, func(path string, d os.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		if d.IsDir() {
			return nil
		}
		name := strings.ToLower(d.Name())
		if !strings.HasSuffix(name, "zones.json") {
			return nil
		}
		info, err := d.Info()
		if err != nil {
			return err
		}
		if best.path == "" || info.ModTime().UnixNano() > best.modTime {
			best.path = path
			best.modTime = info.ModTime().UnixNano()
		}
		return nil
	})
	if err != nil {
		return "", err
	}
	if best.path == "" {
		return "", fmt.Errorf("no *zones.json under %q", dir)
	}
	return best.path, nil
}

func runDisplay(flags DisplayFlags) error {
	showZones := flags.ShowZones || flags.ShowAll

	files, err := listGeometryInDir(flags.LocalFiles)
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

	var zonesPath string
	if showZones {
		zp, err := findNewestZonesJSON(flags.LocalFiles)
		if err != nil {
			return err
		}
		zonesPath = zp
	}

	if len(filtered) == 0 && zonesPath == "" {
		return fmt.Errorf("no .pcd, .ply, .stl, or *zones.json to display (check filters and --local-files %q)", flags.LocalFiles)
	}

	vizClient.SetURL(flags.VizURL)
	if flags.ClearFirst {
		if err := vizClient.RemoveAllSpatialObjects(); err != nil {
			return fmt.Errorf("failed to clear visualizer objects: %w", err)
		}
	}

	if len(filtered) > 0 {
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
	}

	if showZones {
		result, err := segmentation.LoadZones(zonesPath)
		if err != nil {
			return err
		}
		if len(result.Zones) == 0 {
			return fmt.Errorf("zones file %q has no zones", zonesPath)
		}
		if len(filtered) == 0 {
			fmt.Printf("Drawing zones from %s to %s\n", zonesPath, flags.VizURL)
			if err := drawFullZonesViz(zoneVizOptions{
				ClearFirst: false, VizURL: flags.VizURL, LocalDir: flags.LocalFiles, PreferredMesh: "", Result: result,
			}); err != nil {
				return err
			}
		} else {
			fmt.Printf("Appending %d zone(s) from %s\n", len(result.Zones), zonesPath)
			if err := appendZoneColoredMeshes(result); err != nil {
				return err
			}
		}
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

// listGeometryInDir lists .pcd, .ply, and .stl under dir (by modified time, then path). May be empty.
func listGeometryInDir(dir string) ([]displayFile, error) {
	if dir == "" {
		return nil, fmt.Errorf("--local-files directory is required")
	}
	info, err := os.Stat(dir)
	if err != nil {
		return nil, fmt.Errorf("failed to access --local-files %q: %w", dir, err)
	}
	if !info.IsDir() {
		return nil, fmt.Errorf("--local-files must be a directory, got %q", dir)
	}
	var matches []displayFile
	err = filepath.WalkDir(dir, func(path string, d os.DirEntry, walkErr error) error {
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
		return nil, fmt.Errorf("failed to scan folder %q: %w", dir, err)
	}
	sort.Slice(matches, func(i, j int) bool {
		if matches[i].modTime == matches[j].modTime {
			return matches[i].path < matches[j].path
		}
		return matches[i].modTime < matches[j].modTime
	})
	return matches, nil
}
