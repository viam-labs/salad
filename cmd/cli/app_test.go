package main

import (
	"context"
	"reflect"
	"strings"
	"testing"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"

	"salad"
	"salad/events"
)

// TestParseBuild covers the build payload parsing. The critical contract: the
// payload is map[string]interface{} with float64 values, matching what the
// coordinator type-asserts over gRPC (a regression we hit once).
func TestParseBuild(t *testing.T) {
	cases := []struct {
		name, in     string
		wantCustomer string
		wantPayload  map[string]float64
		wantErr      bool
	}{
		{"kv", "lettuce=2 chicken=1", "", map[string]float64{"lettuce": 2, "chicken": 1}, false},
		{"kv-customer", "lettuce=1 customer=alice", "alice", map[string]float64{"lettuce": 1}, false},
		{"kv-customer_name", "lettuce=1 customer_name=bob", "bob", map[string]float64{"lettuce": 1}, false},
		{"json", `{"lettuce":2}`, "", map[string]float64{"lettuce": 2}, false},
		{"json-customer", `{"lettuce":1,"customer":"bob"}`, "bob", map[string]float64{"lettuce": 1}, false},
		{"empty", "", "", nil, true},
		{"no-eq", "lettuce", "", nil, true},
		{"not-a-number", "lettuce=abc", "", nil, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			payload, customer, err := parseBuild(tc.in)
			if tc.wantErr {
				if err == nil {
					t.Fatalf("expected error for %q", tc.in)
				}
				return
			}
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if customer != tc.wantCustomer {
				t.Errorf("customer = %q, want %q", customer, tc.wantCustomer)
			}
			if len(payload) != len(tc.wantPayload) {
				t.Fatalf("payload = %v, want %v", payload, tc.wantPayload)
			}
			for k, want := range tc.wantPayload {
				v, ok := payload[k]
				if !ok {
					t.Errorf("missing ingredient %q", k)
					continue
				}
				f, ok := v.(float64) // the coordinator asserts float64 values
				if !ok {
					t.Errorf("ingredient %q is %T, want float64", k, v)
					continue
				}
				if f != want {
					t.Errorf("ingredient %q = %v, want %v", k, f, want)
				}
			}
		})
	}
}

func TestShortDepName(t *testing.T) {
	cases := map[string]string{
		"generic/salad-coordinator":       "salad-coordinator",
		"left-arm":                        "left-arm",
		"rdk:service:generic/bin-grabber": "bin-grabber",
		"frame_system/$framesystem":       "$framesystem",
	}
	for in, want := range cases {
		if got := shortDepName(in); got != want {
			t.Errorf("shortDepName(%q) = %q, want %q", in, got, want)
		}
	}
}

// stubValidator lets topo/dep tests control the deps a config reports.
type stubValidator struct{ deps []string }

func (s stubValidator) Validate(string) ([]string, []string, error) { return s.deps, nil, nil }

func TestSaladDepNames(t *testing.T) {
	c := resource.Config{Name: "coordinator", ConvertedAttributes: stubValidator{deps: []string{
		"generic/grabber", "rdk:service:generic/dressing", "left-arm", "grabber", // dup + non-salad
	}}}
	names := map[string]bool{"grabber": true, "dressing": true, "coordinator": true}
	got := saladDepNames(c, names)
	want := []string{"grabber", "dressing"} // deduped, non-salad "left-arm" excluded
	if !reflect.DeepEqual(got, want) {
		t.Errorf("saladDepNames = %v, want %v", got, want)
	}
}

// TestSaladDepNamesRealCoordinator is a contract test: the real coordinator
// config must report its sub-services as deps (and not hardware).
func TestSaladDepNamesRealCoordinator(t *testing.T) {
	cfg := &salad.BuildCoordinatorConfig{
		GrabberControls:   "bin-grabber",
		DressingControls:  "dressing-controls",
		ChefsKissControls: "chefs-kiss-controls",
		ScaleSensor:       "scale",
		LeftHome:          "left-home",
		RightHome:         "right-home",
		Ingredients: []salad.BuildCoordinatorIngredientConfig{
			{Name: "romaine", GramsPerServing: 20, Category: "base", ZoneID: 0},
		},
	}
	c := resource.Config{Name: "salad-coordinator", ConvertedAttributes: cfg}
	names := map[string]bool{"bin-grabber": true, "dressing-controls": true, "chefs-kiss-controls": true}
	got := saladDepNames(c, names)
	want := map[string]bool{"bin-grabber": true, "dressing-controls": true, "chefs-kiss-controls": true}
	if len(got) != len(want) {
		t.Fatalf("saladDepNames = %v, want the 3 sub-services", got)
	}
	for _, n := range got {
		if !want[n] {
			t.Errorf("unexpected dep %q (hardware should be excluded)", n)
		}
	}
}

func TestTopoSortSalad(t *testing.T) {
	mk := func(name string, deps ...string) resource.Config {
		return resource.Config{Name: name, ConvertedAttributes: stubValidator{deps: deps}}
	}
	cfgs := []resource.Config{
		mk("grabber"),
		mk("dressing"),
		mk("chefs"),
		mk("coordinator", "grabber", "dressing", "chefs"),
		mk("maintenance", "coordinator"),
	}
	names := map[string]bool{"grabber": true, "dressing": true, "chefs": true, "coordinator": true, "maintenance": true}
	ordered, err := topoSortSalad(cfgs, names)
	if err != nil {
		t.Fatal(err)
	}
	if len(ordered) != len(cfgs) {
		t.Fatalf("got %d resources, want %d", len(ordered), len(cfgs))
	}
	pos := map[string]int{}
	for i, c := range ordered {
		pos[c.Name] = i
	}
	for _, dep := range []string{"grabber", "dressing", "chefs"} {
		if pos["coordinator"] < pos[dep] {
			t.Errorf("coordinator built before its dep %q", dep)
		}
	}
	if pos["maintenance"] < pos["coordinator"] {
		t.Errorf("maintenance-sensor built before coordinator")
	}
}

func TestTopoSortSaladCycle(t *testing.T) {
	mk := func(name string, deps ...string) resource.Config {
		return resource.Config{Name: name, ConvertedAttributes: stubValidator{deps: deps}}
	}
	cfgs := []resource.Config{mk("a", "b"), mk("b", "a")}
	names := map[string]bool{"a": true, "b": true}
	if _, err := topoSortSalad(cfgs, names); err == nil {
		t.Fatal("expected a cycle error")
	}
}

// TestFormatEventContract builds the real events sensor, emits an event, drains
// it, and formats it — pinning the CLI↔sensor key contract (the event_type vs
// type regression we hit).
func TestFormatEventContract(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)
	reg, ok := resource.LookupRegistration(sensor.API, events.Model)
	if !ok {
		t.Fatal("events sensor model not registered")
	}
	conf := resource.Config{Name: "build-events", API: sensor.API, Model: events.Model, ConvertedAttributes: &events.Config{}}
	res, err := reg.Constructor(ctx, resource.Dependencies{}, conf, logger)
	if err != nil {
		t.Fatalf("construct events sensor: %v", err)
	}
	emitter, ok := res.(events.Emitter)
	if !ok {
		t.Fatal("events sensor does not implement events.Emitter")
	}
	emitter.Emit(ctx, events.Event{Type: events.TypeBuildStart, CustomerName: "x"})

	reading, err := res.(sensor.Sensor).Readings(ctx, nil)
	if err != nil {
		t.Fatalf("Readings: %v", err)
	}
	out := formatEvent(reading)
	if !strings.Contains(out, events.TypeBuildStart) {
		t.Errorf("formatEvent did not render the event type; got %q", out)
	}
}

func TestEmailToName(t *testing.T) {
	cases := map[string]string{
		"clint.purser@viam.com": "clint.purser",
		"noatsign":              "noatsign",
		"":                      "",
		"@x":                    "@x",
	}
	for in, want := range cases {
		if got := emailToName(in); got != want {
			t.Errorf("emailToName(%q) = %q, want %q", in, got, want)
		}
	}
}

func TestResolveCustomer(t *testing.T) {
	if got := resolveCustomer("alice"); got != "alice" {
		t.Errorf("explicit: got %q, want alice", got)
	}
	old := appFlags.Customer
	t.Cleanup(func() { appFlags.Customer = old })
	appFlags.Customer = "bob"
	if got := resolveCustomer(""); got != "bob" {
		t.Errorf("flag default: got %q, want bob", got)
	}
	if got := resolveCustomer("alice"); got != "alice" {
		t.Errorf("explicit should win over flag: got %q", got)
	}
}

func TestSetAssetsDir(t *testing.T) {
	g := &salad.GrabberControlsConfig{}
	setAssetsDir(g, "/work/assets")
	if g.AssetsDir != "/work/assets" {
		t.Errorf("grabber assets-dir = %q, want /work/assets", g.AssetsDir)
	}
	// A config type without an assets dir must be a no-op, not a panic.
	setAssetsDir(&events.Config{}, "/work/assets")
}
