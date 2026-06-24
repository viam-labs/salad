// Package events provides the build-events sensor and an Emitter the salad
// build coordinator uses to publish per-build observability events.
package events

import (
	"context"
	"strings"
	"time"
)

// Event types. Each Event.Type is one of these string constants.
const (
	TypeBuildStart       = "build_start"
	TypeBuildComplete    = "build_complete"
	TypeBuildFailed      = "build_failed"
	TypeBuildStopped     = "build_stopped"
	TypeIngredientStart  = "ingredient_start"
	TypeIngredientFinish = "ingredient_complete"
	TypeGrabAttempt      = "grab_attempt"
	TypeDressingPour     = "dressing_pour"
	TypeSetupComplete    = "setup_complete"
	TypeSetupFailed      = "setup_failed"
)

// Event is one observability record. Common fields are typed; event-specific
// fields go in Fields and are flattened into the top-level reading payload.
type Event struct {
	Type                string
	BuildID             string
	CustomerName        string // normalized (use NormalizeName)
	CustomerNameDisplay string // original-cased, trimmed
	Theme               string
	Fields              map[string]interface{}
	Timestamp           time.Time
}

// Emitter is the dependency the coordinator and related services use to
// publish events. A nil Emitter is not safe — callers should use Nop instead.
type Emitter interface {
	Emit(ctx context.Context, e Event)
}

// Nop discards every event. Use when no build-events sensor is configured.
type Nop struct{}

// Emit implements Emitter.
func (Nop) Emit(context.Context, Event) {}

// NormalizeName produces a stable join key from a customer-entered name:
// trim surrounding whitespace, collapse internal runs of whitespace to a
// single space, and lowercase. Returns "" for empty/whitespace-only input.
func NormalizeName(s string) string {
	fields := strings.Fields(s)
	if len(fields) == 0 {
		return ""
	}
	return strings.ToLower(strings.Join(fields, " "))
}

// DisplayName returns the trimmed name with original casing, suitable for UI
// rendering. Returns "" for empty/whitespace-only input.
func DisplayName(s string) string {
	return strings.TrimSpace(s)
}
