package events

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/data"
)

func newTestSensor(t *testing.T) *buildEventsSensor {
	t.Helper()
	return &buildEventsSensor{}
}

func TestReadingsEmptyReturnsNoCaptureToStore(t *testing.T) {
	s := newTestSensor(t)
	r, err := s.Readings(context.Background(), nil)
	if !errors.Is(err, data.ErrNoCaptureToStore) {
		t.Fatalf("expected ErrNoCaptureToStore, got %v", err)
	}
	if r != nil {
		t.Fatalf("expected nil readings, got %v", r)
	}
}

func TestEmitThenReadingsDrainsOnePerCall(t *testing.T) {
	s := newTestSensor(t)
	ctx := context.Background()
	now := time.Date(2026, 6, 24, 12, 0, 0, 0, time.UTC)

	s.Emit(ctx, Event{
		Type:                TypeBuildStart,
		BuildID:             "build-1",
		CustomerName:        "alice",
		CustomerNameDisplay: "Alice",
		Theme:               "salad",
		Timestamp:           now,
		Fields:              map[string]interface{}{"total_servings": 3.0},
	})
	s.Emit(ctx, Event{
		Type:      TypeBuildComplete,
		BuildID:   "build-1",
		Timestamp: now.Add(time.Minute),
		Fields:    map[string]interface{}{"duration_ms": 60000.0},
	})

	r1, err := s.Readings(ctx, nil)
	if err != nil {
		t.Fatalf("first Readings: %v", err)
	}
	if r1["event_type"] != TypeBuildStart {
		t.Fatalf("expected build_start first, got %v", r1["event_type"])
	}
	if r1["customer_name"] != "alice" || r1["customer_name_display"] != "Alice" {
		t.Fatalf("unexpected customer fields: %v", r1)
	}
	if r1["total_servings"].(float64) != 3.0 {
		t.Fatalf("flattened field missing: %v", r1)
	}
	if r1["timestamp"] != now.Format(time.RFC3339Nano) {
		t.Fatalf("unexpected timestamp: %v", r1["timestamp"])
	}

	r2, err := s.Readings(ctx, nil)
	if err != nil {
		t.Fatalf("second Readings: %v", err)
	}
	if r2["event_type"] != TypeBuildComplete {
		t.Fatalf("expected build_complete second, got %v", r2["event_type"])
	}

	_, err = s.Readings(ctx, nil)
	if !errors.Is(err, data.ErrNoCaptureToStore) {
		t.Fatalf("queue should be drained; got err=%v", err)
	}
}

func TestNormalizeName(t *testing.T) {
	cases := []struct {
		in, want string
	}{
		{"Alice", "alice"},
		{"  Alice  ", "alice"},
		{"Alice  Smith", "alice smith"},
		{"\tAlice\nSmith ", "alice smith"},
		{"", ""},
		{"   ", ""},
		{"ALICE", "alice"},
	}
	for _, c := range cases {
		if got := NormalizeName(c.in); got != c.want {
			t.Errorf("NormalizeName(%q) = %q, want %q", c.in, got, c.want)
		}
	}
}

func TestDisplayName(t *testing.T) {
	if DisplayName("  Alice  ") != "Alice" {
		t.Fail()
	}
	if DisplayName("") != "" {
		t.Fail()
	}
}

func TestNopEmitter(t *testing.T) {
	var e Emitter = Nop{}
	e.Emit(context.Background(), Event{Type: TypeBuildStart})
}
