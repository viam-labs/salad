package main

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"os/signal"
	"path"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"syscall"
	"time"

	"github.com/spf13/cobra"
	"go.viam.com/rdk/app"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/robot/framesystem"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"

	"salad"
	"salad/events"
)

// The "app" command runs the salad application's Go services locally: it dials
// the live machine for hardware, but constructs every salad service (coordinator
// + sub-controls) IN THIS PROCESS using the same constructors cmd/module
// registers. Editing a service and running `go build ./cmd/cli` is then the whole
// feedback loop — no module reload.
//
// Only the salad generic services run locally; arms, cameras, sensors, switches
// and non-salad services (arm-shaker, xarm-force-mover, …) stay on the machine
// and are reached as remote resources.

const (
	// defaultMachine is salad1's machine (robot) id; everything else (part id,
	// address, secret) is derived from it at runtime.
	defaultMachine    = "cf72d260-fb96-4ec9-8f33-c162dd9bf16f"
	defaultAppAddress = "https://app.viam.com:443"
	localEventsName   = "build-events"
)

type appFlagsT struct {
	Machine          string
	AppAddress       string
	WorkDir          string
	MachineAssetsDir string
	Customer         string
	Simulate         bool
	WireEvents       bool
	Exec             string
	Debug            bool
}

var appFlags appFlagsT

var appCmd = &cobra.Command{
	Use:   "app",
	Short: "Run the salad services locally against a live machine (no module reload)",
	Long: "Constructs the salad coordinator and sub-control services in-process,\n" +
		"wired to the live machine's hardware, then drops into a REPL (build, status,\n" +
		"stop, setup, ingredients, do, events). Use --exec to run one command and exit.",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runApp(cmd.Context())
	},
}

func init() {
	appCmd.Flags().StringVar(&appFlags.Machine, "machine", defaultMachine, "machine (robot) id to run against; address, part id and secret are derived from it")
	appCmd.Flags().StringVar(&appFlags.AppAddress, "app-address", defaultAppAddress, "Viam app address")
	appCmd.Flags().StringVar(&appFlags.WorkDir, "work-dir", "./.salad-local", "local dir for the coordinator's assets/captures (keeps it off machine paths); empty = use config paths")
	appCmd.Flags().StringVar(&appFlags.Customer, "customer", "", "default customer name for builds; empty = your viam CLI identity, else \"test\"")
	appCmd.Flags().StringVar(&appFlags.MachineAssetsDir, "machine-assets-dir", "/home/viam/assets", "path on the machine to pull setup assets (merged.pcd, zones.json, mesh.ply) from")
	appCmd.Flags().BoolVar(&appFlags.Simulate, "simulate", false, "run builds without moving hardware (sets the coordinator's simulate flag)")
	appCmd.Flags().BoolVar(&appFlags.WireEvents, "wire-events", true, "construct a local build-events sensor and wire it into the coordinator if it has none")
	appCmd.Flags().StringVar(&appFlags.Exec, "exec", "", "run a single command then exit (e.g. --exec status); omit for an interactive REPL")
	appCmd.Flags().BoolVar(&appFlags.Debug, "debug", false, "verbose rdk logging")
	rootCmd.AddCommand(appCmd)
}

// localTree holds the in-process salad service graph and the live connections
// backing it.
type localTree struct {
	machine     robot.Robot
	viam        *app.ViamClient
	conn        rpc.ClientConn
	logger      logging.Logger
	host        string // resolved machine address
	partID      string // resolved main-part id (for `viam machines part cp`)
	coord       resource.Resource
	coordName   string
	coordRName  resource.Name                // coordinator resource name (to reach the remote one)
	eventsIn    sensor.Sensor                // local build-events sensor, if wired
	resources   map[string]resource.Resource // locally-built salad resources, by name
	models      map[string]string            // name -> model string, for locally-built resources
	localNames  []string                     // salad resources constructed locally
	remoteNames []string                     // salad resources that fell back to remote
}

func runApp(ctx context.Context) error {
	// Two loggers: netLogger is kept quiet (rdk client/networking noise like the
	// SESSION_EXPIRED heartbeat); svcLogger carries the salad services' own logs —
	// the ones you'd otherwise tail on the machine — surfaced locally.
	netLogger := logging.NewLogger("salad-local")
	svcLogger := logging.NewLogger("salad")
	if appFlags.Debug {
		netLogger.SetLevel(logging.DEBUG)
		svcLogger.SetLevel(logging.DEBUG)
	} else {
		netLogger.SetLevel(logging.ERROR)
		svcLogger.SetLevel(logging.INFO)
	}

	tree, err := buildLocalTree(ctx, netLogger, svcLogger)
	if err != nil {
		return err
	}
	defer tree.close(ctx)

	mode := "REAL HARDWARE"
	if appFlags.Simulate {
		mode = "SIMULATE (no motion)"
	}
	fmt.Printf("\nsalad app running locally against %s — %s\n", tree.host, mode)
	fmt.Printf("coordinator: %s\n", tree.coordName)
	if appFlags.WorkDir != "" {
		fmt.Printf("work dir: %s (setup/build assets live here, not on the machine)\n", appFlags.WorkDir)
	}
	fmt.Printf("default customer: %s (override with customer=… on build)\n", resolveCustomer(""))
	if !appFlags.Simulate {
		fmt.Println("⚠  real hardware — the machine's deployed coordinator is also live; don't trigger builds from the web app while driving here.")
	}

	// Graceful interrupt: stop any in-flight build and reset hardware before
	// exiting, so we never leave the arm mid-motion.
	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sigCh
		fmt.Println("\ninterrupted — stopping build and resetting hardware…")
		stopCtx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		defer cancel()
		if tree.coord != nil {
			_, _ = tree.coord.DoCommand(stopCtx, map[string]interface{}{"stop": true}) //nolint:errcheck // best-effort stop on interrupt before exit
		}
		tree.close(stopCtx)
		os.Exit(130)
	}()

	if appFlags.Exec != "" {
		out, err := tree.do(ctx, appFlags.Exec)
		if out != "" {
			fmt.Println(out)
		}
		// Flush any events the command produced.
		for _, e := range tree.drainEvents(ctx) {
			fmt.Println(formatEvent(e))
		}
		return err
	}

	return tree.repl(ctx)
}

func buildLocalTree(ctx context.Context, netLogger, svcLogger logging.Logger) (*localTree, error) {
	t := &localTree{logger: netLogger, resources: map[string]resource.Resource{}, models: map[string]string{}}

	// 1. Resolve the target machine's main part (address, part id, secret) from
	//    its robot id, using the `viam login` token.
	partID, host, secret, err := t.resolveMachine(ctx)
	if err != nil {
		t.close(ctx)
		return nil, err
	}
	t.host = host
	t.partID = partID

	// 2. Live machine -> dependency map (hardware + non-salad services, remote).
	fmt.Printf("dialing %s …\n", host)
	machine, err := connectMachineFromCLIToken(ctx, host, netLogger)
	if err != nil {
		t.close(ctx)
		return nil, fmt.Errorf("connect to machine (run `viam login`?): %w", err)
	}
	t.machine = machine
	deps, err := machineToDependencies(machine)
	if err != nil {
		t.close(ctx)
		return nil, fmt.Errorf("build dependencies from machine: %w", err)
	}

	// 3. Merged config (all fragments resolved), the same way viam-server boots.
	cfg, err := t.fetchMergedConfig(ctx, partID, secret)
	if err != nil {
		t.close(ctx)
		return nil, err
	}

	// 4. Identify every salad module resource in the merged config.
	saladCfgs := collectSaladResources(cfg)
	saladNames := map[string]bool{}
	hasEvents := false
	for _, c := range saladCfgs {
		saladNames[c.Name] = true
		if c.Model.String() == events.Model.String() {
			hasEvents = true
		}
	}

	// 5. Events are debug-only: drained in-process and printed, never captured by
	//    a data manager (we construct none) and never synced. If the machine has
	//    no build-events sensor, wire a local in-memory one for the coordinator.
	eventsName := ""
	if hasEvents {
		for _, c := range saladCfgs {
			if c.Model.String() == events.Model.String() {
				eventsName = c.Name
			}
		}
	} else if appFlags.WireEvents {
		ec := resource.Config{Name: localEventsName, API: sensor.API, Model: events.Model, ConvertedAttributes: &events.Config{}}
		evRes, err := construct(ctx, deps, ec, svcLogger.Sublogger(localEventsName))
		if err != nil {
			t.close(ctx)
			return nil, fmt.Errorf("build local build-events sensor: %w", err)
		}
		t.eventsIn = evRes.(sensor.Sensor)
		deps[ec.ResourceName()] = evRes
		t.resources[localEventsName] = evRes
		t.models[localEventsName] = events.Model.String()
		eventsName = localEventsName
		fmt.Printf("built local %-22s (model %s)\n", localEventsName, events.Model)
	}

	// 6. Construct every salad resource locally in dependency order. Non-salad
	//    deps (arms, cameras, switches, …) stay remote. Best-effort: if a
	//    non-coordinator resource can't build locally (e.g. file-vision needs a
	//    machine-side mesh file), keep the remote one and continue.
	ordered, err := topoSortSalad(saladCfgs, saladNames)
	if err != nil {
		t.close(ctx)
		return nil, err
	}
	for _, c := range ordered {
		c.AssociatedAttributes = nil // never run data capture locally
		// Point every salad service that reads setup artifacts (coordinator,
		// grabber, dressing) at the local work-dir — filled with the machine's
		// real assets — instead of /home/viam.
		if appFlags.WorkDir != "" {
			setAssetsDir(c.ConvertedAttributes, filepath.Join(appFlags.WorkDir, "assets"))
		}
		isCoord := c.Model.String() == salad.BuildCoordinator.String()
		if isCoord {
			if bc, ok := c.ConvertedAttributes.(*salad.BuildCoordinatorConfig); ok {
				if appFlags.Simulate {
					bc.Simulate = true
				}
				if appFlags.WireEvents && bc.BuildEvents == "" {
					bc.BuildEvents = eventsName
				}
				if appFlags.WorkDir != "" {
					bc.CaptureDir = filepath.Join(appFlags.WorkDir, "capture")
				}
			}
		}
		res, err := construct(ctx, deps, c, svcLogger.Sublogger(c.Name))
		if err != nil {
			if isCoord {
				t.close(ctx)
				return nil, fmt.Errorf("build local coordinator %q: %w", c.Name, err)
			}
			fmt.Printf("  ! %-22s could not build locally (%v) — using remote\n", c.Name, err)
			t.remoteNames = append(t.remoteNames, c.Name)
			continue
		}
		deps[c.ResourceName()] = res
		t.resources[c.Name] = res
		t.models[c.Name] = c.Model.String()
		t.localNames = append(t.localNames, c.Name)
		if isCoord {
			t.coord = res
			t.coordName = c.Name
			t.coordRName = c.ResourceName()
		}
		if s, ok := res.(sensor.Sensor); ok && c.Model.String() == events.Model.String() {
			t.eventsIn = s
		}
		fmt.Printf("built local %-22s (model %s)\n", c.Name, c.Model)
	}
	if t.coord == nil {
		t.close(ctx)
		return nil, errors.New("no salad build-coordinator found in the machine config")
	}
	return t, nil
}

// resolveMachine looks up the machine's main part from its robot id and returns
// the part id, address, and secret needed to dial the machine and fetch its
// config — so the caller only ever passes a robot id.
func (t *localTree) resolveMachine(ctx context.Context) (partID, host, secret string, err error) {
	viamClient, err := app.ConnectFromCLIToken(ctx, t.logger)
	if err != nil {
		return "", "", "", fmt.Errorf("connect to app (run `viam login`?): %w", err)
	}
	t.viam = viamClient
	parts, err := viamClient.AppClient().GetRobotParts(ctx, appFlags.Machine)
	if err != nil {
		return "", "", "", fmt.Errorf("list parts for machine %q: %w", appFlags.Machine, err)
	}
	if len(parts) == 0 {
		return "", "", "", fmt.Errorf("machine %q has no parts", appFlags.Machine)
	}
	main := parts[0]
	for _, p := range parts {
		if p.MainPart {
			main = p
			break
		}
	}
	// Re-fetch the part by id so we reliably get its secret.
	part, _, err := viamClient.AppClient().GetRobotPart(ctx, main.ID)
	if err != nil {
		return "", "", "", fmt.Errorf("get part %q: %w", main.ID, err)
	}
	if part.FQDN == "" {
		return "", "", "", fmt.Errorf("part %q has no address", part.ID)
	}
	return part.ID, part.FQDN, part.Secret, nil
}

// fetchMergedConfig returns the machine's fully fragment-merged config, with each
// resource's attributes converted to its typed config. Uses the part secret to
// authenticate the cloud config fetch, exactly like viam-server on boot.
func (t *localTree) fetchMergedConfig(ctx context.Context, partID, secret string) (*config.Config, error) {
	cloudCfg := &config.Cloud{ID: partID, Secret: secret, AppAddress: appFlags.AppAddress}
	conn, err := config.CreateNewGRPCClient(ctx, cloudCfg, t.logger)
	if err != nil {
		return nil, fmt.Errorf("dial app for config: %w", err)
	}
	t.conn = conn
	cloudJSON := fmt.Sprintf(`{"cloud":{"id":%q,"secret":%q,"app_address":%q}}`,
		partID, secret, appFlags.AppAddress)
	cfg, err := config.FromReader(ctx, "", strings.NewReader(cloudJSON), t.logger, conn)
	if err != nil {
		return nil, fmt.Errorf("read merged config: %w", err)
	}
	return cfg, nil
}

func (t *localTree) close(ctx context.Context) {
	if t.coord != nil {
		_ = t.coord.Close(ctx) //nolint:errcheck // best-effort cleanup on shutdown
	}
	if t.eventsIn != nil {
		_ = t.eventsIn.Close(ctx) //nolint:errcheck // best-effort cleanup on shutdown
	}
	if t.conn != nil {
		_ = t.conn.Close() //nolint:errcheck // best-effort cleanup on shutdown
	}
	if t.viam != nil {
		_ = t.viam.Close() //nolint:errcheck // best-effort cleanup on shutdown
	}
	if t.machine != nil {
		_ = t.machine.Close(ctx) //nolint:errcheck // best-effort cleanup on shutdown
	}
}

// connectMachineFromCLIToken dials a live machine using the `viam login` token
// cache, so no API keys are needed in code.
func connectMachineFromCLIToken(ctx context.Context, host string, logger logging.Logger) (robot.Robot, error) {
	conf, err := rdkutils.ConfigFromPath(rdkutils.GetCLICachePath())
	if err != nil {
		return nil, err
	}
	dopts, err := conf.DialOptions()
	if err != nil {
		return nil, err
	}
	return client.New(ctx, host, logger, client.WithDialOptions(dopts...))
}

// machineToDependencies turns every resource on the live machine into a
// dependency map and adds the frame system, so locally-constructed services can
// bind to remote hardware.
func machineToDependencies(r robot.Robot) (resource.Dependencies, error) {
	deps := resource.Dependencies{}
	for _, n := range r.ResourceNames() {
		res, err := r.ResourceByName(n)
		if err != nil {
			return nil, err
		}
		deps[n] = res
	}
	rr, ok := r.(resource.Resource)
	if !ok {
		return nil, errors.New("robot client is not a resource.Resource")
	}
	deps[framesystem.PublicServiceName] = rr
	return deps, nil
}

// construct builds a resource from its config via the resource registry, using
// the same constructor cmd/module registers.
func construct(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (resource.Resource, error) {
	reg, ok := resource.LookupRegistration(conf.API, conf.Model)
	if !ok {
		return nil, fmt.Errorf("no registration for %s/%s (is the salad package imported?)", conf.API, conf.Model)
	}
	if reg.Constructor == nil {
		return nil, fmt.Errorf("registration for %s/%s has no constructor", conf.API, conf.Model)
	}
	return reg.Constructor(ctx, deps, conf, logger)
}

// setAssetsDir points a salad service's config at the given assets dir, for the
// services that read setup artifacts (merged.pcd, zones.json, mesh.ply).
func setAssetsDir(attrs interface{}, dir string) {
	switch cfg := attrs.(type) {
	case *salad.BuildCoordinatorConfig:
		cfg.AssetsDir = dir
	case *salad.GrabberControlsConfig:
		cfg.AssetsDir = dir
	case *salad.DressingControlsConfig:
		cfg.AssetsDir = dir
	}
}

// collectSaladResources returns every resource in the merged config whose model
// is a salad module model, across all APIs (generic, vision, sensor).
func collectSaladResources(cfg *config.Config) []resource.Config {
	var out []resource.Config
	for _, list := range [][]resource.Config{cfg.Services, cfg.Components} {
		for _, c := range list {
			if strings.HasPrefix(c.Model.String(), "ncs:salad:") {
				out = append(out, c)
			}
		}
	}
	return out
}

// shortDepName extracts the resource short name from a dependency string, which
// may be "name", "api/name", or "ns:type:sub/name".
func shortDepName(dep string) string {
	if i := strings.LastIndex(dep, "/"); i >= 0 {
		return dep[i+1:]
	}
	return dep
}

// saladDepNames returns the deduped names of c's dependencies that are themselves
// salad resources, so they get constructed before c.
func saladDepNames(c resource.Config, saladNames map[string]bool) []string {
	v := c.ConvertedAttributes
	if v == nil {
		return nil
	}
	req, opt, err := v.Validate("")
	if err != nil {
		return nil
	}
	seen := map[string]bool{}
	var deps []string
	for _, d := range append(append([]string{}, req...), opt...) {
		n := shortDepName(d)
		if saladNames[n] && n != c.Name && !seen[n] {
			seen[n] = true
			deps = append(deps, n)
		}
	}
	return deps
}

// topoSortSalad orders salad configs so each is constructed after the salad
// resources it depends on (e.g. coordinator after its sub-services,
// maintenance-sensor after the coordinator). Returns an error on a cycle.
func topoSortSalad(cfgs []resource.Config, saladNames map[string]bool) ([]resource.Config, error) {
	byName := map[string]resource.Config{}
	indeg := map[string]int{}
	for _, c := range cfgs {
		byName[c.Name] = c
		indeg[c.Name] = 0
	}
	deps := map[string][]string{}
	for _, c := range cfgs {
		for _, d := range saladDepNames(c, saladNames) {
			if _, ok := byName[d]; ok {
				deps[c.Name] = append(deps[c.Name], d)
				indeg[c.Name]++
			}
		}
	}
	var queue []string
	for _, c := range cfgs { // preserve config order among independent nodes
		if indeg[c.Name] == 0 {
			queue = append(queue, c.Name)
		}
	}
	var ordered []resource.Config
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		ordered = append(ordered, byName[cur])
		for _, c := range cfgs {
			for _, d := range deps[c.Name] {
				if d == cur {
					indeg[c.Name]--
					if indeg[c.Name] == 0 {
						queue = append(queue, c.Name)
					}
				}
			}
		}
	}
	if len(ordered) != len(cfgs) {
		return nil, fmt.Errorf("dependency cycle among salad resources (ordered %d/%d)", len(ordered), len(cfgs))
	}
	return ordered, nil
}

// repl reads commands from stdin and drives the local coordinator. A background
// ticker prints build-events as the coordinator emits them.
func (t *localTree) repl(ctx context.Context) error {
	t.printIngredients(ctx)
	if t.eventsIn != nil {
		go t.watchEvents(ctx)
	}
	fmt.Println("\n" + helpText)
	sc := bufio.NewScanner(os.Stdin)
	fmt.Print("\nsalad> ")
	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "quit" || line == "exit" {
			break
		}
		if line != "" {
			out, err := t.do(ctx, line)
			if out != "" {
				fmt.Println(out)
			}
			if err != nil {
				fmt.Println("error:", err)
			}
		}
		fmt.Print("\nsalad> ")
	}
	return sc.Err()
}

func (t *localTree) watchEvents(ctx context.Context) {
	tick := time.NewTicker(400 * time.Millisecond)
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
			for _, e := range t.drainEvents(ctx) {
				fmt.Printf("\n%s\nsalad> ", formatEvent(e))
			}
		}
	}
}

func (t *localTree) drainEvents(ctx context.Context) []map[string]interface{} {
	if t.eventsIn == nil {
		return nil
	}
	var out []map[string]interface{}
	for {
		reading, err := t.eventsIn.Readings(ctx, nil)
		if err != nil {
			// ErrNoCaptureToStore means the queue is empty; anything else we stop on.
			return out
		}
		if reading == nil {
			return out
		}
		out = append(out, reading)
	}
}

// fetchAssets copies the machine's REAL setup artifacts (merged.pcd, zones.json,
// mesh.ply) into the local work-dir via the machine's shell service, so a local
// build runs on real machine data — no locally-generated assets and no module
// deploy needed. Requires `viam` on PATH and a shell service on the machine.
func (t *localTree) fetchAssets(ctx context.Context) error {
	if appFlags.WorkDir == "" {
		return errors.New("--work-dir is empty; nowhere to fetch assets into")
	}
	dir := filepath.Join(appFlags.WorkDir, "assets")
	if err := os.MkdirAll(dir, 0o750); err != nil {
		return err
	}
	for _, name := range []string{"merged.pcd", "zones.json", "mesh.ply"} {
		src := "machine:" + path.Join(appFlags.MachineAssetsDir, name)
		out, err := exec.CommandContext(ctx, "viam", "machines", "part", "cp", //nolint:gosec // fixed args; partID and paths derived from our own config
			"--part", t.partID, src, filepath.Join(dir, name)).CombinedOutput()
		if err != nil {
			return fmt.Errorf("copy %s from machine (needs `viam login` + a shell service; has setup_station run there?): %w\n%s", name, err, out)
		}
	}
	fmt.Printf("fetched real setup assets from machine:%s → %s\n", appFlags.MachineAssetsDir, dir)
	return nil
}

// ensureAssets pulls setup assets from the machine if the local work-dir lacks
// them, so `build` transparently runs on real machine data.
func (t *localTree) ensureAssets(ctx context.Context) error {
	if appFlags.WorkDir == "" {
		return nil // using config paths; leave assets as configured
	}
	dir := filepath.Join(appFlags.WorkDir, "assets")
	_, ePCD := os.Stat(filepath.Join(dir, "merged.pcd"))
	_, eZones := os.Stat(filepath.Join(dir, "zones.json"))
	_, eMesh := os.Stat(filepath.Join(dir, "mesh.ply"))
	if ePCD == nil && eZones == nil && eMesh == nil {
		return nil
	}
	fmt.Println("local setup assets missing — pulling the machine's real ones…")
	return t.fetchAssets(ctx)
}

// printIngredients lists the available ingredient names (grouped by category) so
// you know what you can build with. Best-effort; skipped on error.
func (t *localTree) printIngredients(ctx context.Context) {
	res, err := t.coord.DoCommand(ctx, map[string]interface{}{"list_ingredients": true})
	if err != nil {
		return
	}
	list, _ := res["ingredients"].([]interface{})
	byCat := map[string][]string{}
	for _, item := range list {
		m, _ := item.(map[string]interface{})
		name, _ := m["name"].(string)
		cat, _ := m["category"].(string)
		if name != "" {
			byCat[cat] = append(byCat[cat], name)
		}
	}
	if len(byCat) == 0 {
		return
	}
	fmt.Println("\ningredients:")
	printed := map[string]bool{}
	order := []string{"base", "protein", "topping", "dressing"}
	for cat := range byCat {
		if !contains(order, cat) {
			order = append(order, cat)
		}
	}
	for _, cat := range order {
		names := byCat[cat]
		if len(names) == 0 || printed[cat] {
			continue
		}
		printed[cat] = true
		sort.Strings(names)
		fmt.Printf("  %-9s %s\n", cat+":", strings.Join(names, ", "))
	}
}

func contains(s []string, v string) bool {
	for _, x := range s {
		if x == v {
			return true
		}
	}
	return false
}

const helpText = "commands: build <lettuce=2 chicken=1 [customer=Alice]> | status | stop | setup | " +
	"ingredients | theme | do [<resource>] <json> | resources | fetch-assets | events | help | quit"

// verbHints lists the DoCommand verbs each salad model accepts, shown by the
// `resources` command so you know what to send via `do <resource> {…}`.
var verbHints = map[string][]string{
	salad.BuildCoordinator.String():  {"build_salad", "status", "stop", "setup_station", "list_ingredients", "get_theme", "get_setup_result"},
	salad.GrabberControls.String():   {"calibrate", "get_from_bin", "get_zone_config", "get_gripper_calibration", "reset"},
	salad.DressingControls.String():  {"get_dressings", "pre_plan_dressing", "pour_dressing", "reset"},
	salad.ChefsKissControls.String(): {"chefs_kiss"},
	salad.BowlControls.String():      {"prepare_bowl", "grab_bowl", "grab_lid", "deliver_bowl", "force_move", "use_tool", "reset"},
	salad.SupplyDetector.String():    {"get_readings"},
}

// do dispatches a single REPL/exec command and returns printable output.
func (t *localTree) do(ctx context.Context, line string) (string, error) {
	fields := strings.Fields(line)
	if len(fields) == 0 {
		return "", nil
	}
	verb, rest := fields[0], strings.TrimSpace(strings.TrimPrefix(line, fields[0]))

	target := t.coord // default: drive the coordinator
	var cmd map[string]interface{}
	switch verb {
	case "help":
		return helpText, nil
	case "resources":
		return t.resourceList(), nil
	case "fetch-assets":
		if err := t.fetchAssets(ctx); err != nil {
			return "", err
		}
		return "assets fetched from machine", nil
	case "events":
		evs := t.drainEvents(ctx)
		if len(evs) == 0 {
			return "(no pending events)", nil
		}
		var b strings.Builder
		for _, e := range evs {
			b.WriteString(formatEvent(e))
			b.WriteByte('\n')
		}
		return strings.TrimRight(b.String(), "\n"), nil
	case "status":
		cmd = map[string]interface{}{"status": true}
	case "stop":
		cmd = map[string]interface{}{"stop": true}
	case "setup":
		cmd = map[string]interface{}{"setup_station": true}
	case "ingredients":
		cmd = map[string]interface{}{"list_ingredients": true}
	case "theme":
		cmd = map[string]interface{}{"get_theme": true}
	case "do":
		// `do {json}` drives the coordinator; `do <resource> {json}` drives any
		// locally-built salad resource by name.
		jsonPart := rest
		if !strings.HasPrefix(rest, "{") {
			name := fields[1]
			r, ok := t.resources[name]
			if !ok {
				return "", fmt.Errorf("no local resource %q (try: resources)", name)
			}
			target = r
			jsonPart = strings.TrimSpace(strings.TrimPrefix(rest, name))
		}
		if err := json.Unmarshal([]byte(jsonPart), &cmd); err != nil {
			return "", fmt.Errorf("do [<resource>] <json>: %w", err)
		}
	case "build":
		if err := t.ensureAssets(ctx); err != nil {
			return "", err
		}
		payload, customer, err := parseBuild(rest)
		if err != nil {
			return "", err
		}
		// The CLI doesn't care about real customer names; default to your viam
		// identity (or "test") so you never have to type one.
		cmd = map[string]interface{}{"build_salad": payload, "customer_name": resolveCustomer(customer)}
	default:
		return "", fmt.Errorf("unknown command %q (try: help)", verb)
	}

	res, err := target.DoCommand(ctx, cmd)
	if err != nil {
		return "", err
	}
	return prettyJSON(res), nil
}

// resourceList renders which salad resources are running locally (with their
// DoCommand verbs) vs. which fell back to the real machine.
func (t *localTree) resourceList() string {
	var b strings.Builder
	b.WriteString("local salad resources (drive with `do <name> {json}`):\n")
	names := append([]string{}, t.localNames...)
	sort.Strings(names)
	for _, n := range names {
		model := t.models[n]
		b.WriteString(fmt.Sprintf("  %-22s %-28s verbs: %s\n", n, model, strings.Join(verbHints[model], ", ")))
	}
	if len(t.remoteNames) > 0 {
		sort.Strings(t.remoteNames)
		b.WriteString("remote (could not build locally): " + strings.Join(t.remoteNames, ", ") + "\n")
	}
	return strings.TrimRight(b.String(), "\n")
}

// parseBuild accepts either JSON (`{"lettuce":2}`) or `k=v` pairs
// (`lettuce=2 chicken=1 customer=Alice`). The reserved key `customer` becomes
// the customer name; all other values are serving counts. Returns the payload as
// map[string]interface{} (float64 values) to match what the coordinator
// type-asserts — the same shape the gRPC path delivers.
func parseBuild(rest string) (map[string]interface{}, string, error) {
	rest = strings.TrimSpace(rest)
	if rest == "" {
		return nil, "", errors.New("build needs ingredients, e.g. build lettuce=2 chicken=1")
	}
	payload := map[string]interface{}{}
	var customer string
	if strings.HasPrefix(rest, "{") {
		raw := map[string]interface{}{}
		if err := json.Unmarshal([]byte(rest), &raw); err != nil {
			return nil, "", fmt.Errorf("build <json>: %w", err)
		}
		for k, v := range raw {
			if k == "customer" || k == "customer_name" {
				customer = fmt.Sprint(v)
				continue
			}
			n, ok := v.(float64)
			if !ok {
				return nil, "", fmt.Errorf("ingredient %q must be a number", k)
			}
			payload[k] = n
		}
		return payload, customer, nil
	}
	for _, tok := range strings.Fields(rest) {
		k, v, ok := strings.Cut(tok, "=")
		if !ok {
			return nil, "", fmt.Errorf("expected key=value, got %q", tok)
		}
		if k == "customer" || k == "customer_name" {
			customer = v
			continue
		}
		n, err := strconv.ParseFloat(v, 64)
		if err != nil {
			return nil, "", fmt.Errorf("ingredient %q value %q is not a number", k, v)
		}
		payload[k] = n
	}
	return payload, customer, nil
}

// resolveCustomer picks the customer name for a build: explicit customer=… on the
// build command wins, then the --customer flag, then the viam CLI identity, then
// "test". The CLI doesn't track real customers — this just satisfies the
// coordinator's required name.
func resolveCustomer(explicit string) string {
	if explicit != "" {
		return explicit
	}
	if appFlags.Customer != "" {
		return appFlags.Customer
	}
	if id := cliIdentity(); id != "" {
		return id
	}
	return "test"
}

// cliIdentity returns the viam CLI user's name (local part of their email) from
// the login token cache, or "" if unavailable.
func cliIdentity() string {
	b, err := os.ReadFile(rdkutils.GetCLICachePath())
	if err != nil {
		return ""
	}
	var c struct {
		Auth struct {
			UserData struct {
				Email string `json:"email"`
			} `json:"user_data"`
		} `json:"auth"`
	}
	if json.Unmarshal(b, &c) != nil {
		return ""
	}
	return emailToName(c.Auth.UserData.Email)
}

// emailToName returns the local part of an email (before "@"), or the input
// unchanged if it has no "@".
func emailToName(email string) string {
	if i := strings.Index(email, "@"); i > 0 {
		return email[:i]
	}
	return email
}

func formatEvent(e map[string]interface{}) string {
	typ, _ := e["event_type"].(string)
	keys := make([]string, 0, len(e))
	for k := range e {
		if k == "event_type" {
			continue
		}
		keys = append(keys, k)
	}
	sort.Strings(keys)
	parts := make([]string, 0, len(keys))
	for _, k := range keys {
		parts = append(parts, fmt.Sprintf("%s=%v", k, e[k]))
	}
	return fmt.Sprintf("📡 %-20s %s", typ, strings.Join(parts, " "))
}

func prettyJSON(v interface{}) string {
	b, err := json.MarshalIndent(v, "", "  ")
	if err != nil {
		return fmt.Sprint(v)
	}
	return string(b)
}
