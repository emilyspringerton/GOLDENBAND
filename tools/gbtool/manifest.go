// manifest.go — the .gband.json sidecar manifest (format/GBAND_FORMAT.md).
package main

import (
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
)

// Manifest mirrors format/GBAND_FORMAT.md's manifest schema exactly.
type Manifest struct {
	GBandVersion  int            `json:"gband_version"`
	SkeletonHash  string         `json:"skeleton_hash"`
	ContentHash   string         `json:"content_hash"`
	TickRate      uint32         `json:"tick_rate"`
	DurationTicks uint32         `json:"duration_ticks"`
	Channels      []string       `json:"channels"`
	Authorship    Authorship     `json:"authorship"`
	IntentTags    []string       `json:"intent_tags"`
	LoopPoints    LoopPoints     `json:"loop_points"`
	Safety        SafetyAnnotate `json:"safety"`
}

type Authorship struct {
	Kind string `json:"kind"` // mocap | human | generative
	Who  string `json:"who"`
}

type LoopPoints struct {
	StartTick uint32 `json:"start_tick"`
	EndTick   uint32 `json:"end_tick"`
}

type SafetyAnnotate struct {
	MaxJointVelocity *float64 `json:"max_joint_velocity"`
	MaxJointTorque   *float64 `json:"max_joint_torque"`
}

// WriteManifest writes m to path as indented JSON.
func WriteManifest(path string, m *Manifest) error {
	b, err := json.MarshalIndent(m, "", "  ")
	if err != nil {
		return err
	}
	b = append(b, '\n')
	return os.WriteFile(path, b, 0644)
}

// ReadManifest reads and parses a .gband.json manifest.
func ReadManifest(path string) (*Manifest, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var m Manifest
	if err := json.Unmarshal(raw, &m); err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	return &m, nil
}

// ValidateAgainst cross-checks the manifest against the binary asset it
// describes, per format/GBAND_FORMAT.md's validation rules. Returns a
// list of problems; empty means clean.
func (m *Manifest) ValidateAgainst(g *GBandFile) []string {
	var problems []string

	if uint32(len(m.Channels)) != g.NumChannels {
		problems = append(problems, fmt.Sprintf("manifest declares %d channels, binary has %d", len(m.Channels), g.NumChannels))
	}
	if m.TickRate != g.TickRate {
		problems = append(problems, fmt.Sprintf("manifest tick_rate=%d, binary tick_rate=%d", m.TickRate, g.TickRate))
	}
	if m.DurationTicks != g.DurationTicks {
		problems = append(problems, fmt.Sprintf("manifest duration_ticks=%d, binary duration_ticks=%d", m.DurationTicks, g.DurationTicks))
	}

	wantContentHash := hex.EncodeToString(g.ContentHash[:])
	if m.ContentHash != wantContentHash {
		problems = append(problems, fmt.Sprintf("manifest content_hash=%s does not match binary's actual content_hash=%s", m.ContentHash, wantContentHash))
	}
	if !g.Verify() {
		problems = append(problems, "binary's own content_hash does not match its channel data (corrupt or tampered file)")
	}

	if m.LoopPoints.EndTick > g.DurationTicks {
		problems = append(problems, fmt.Sprintf("loop_points.end_tick=%d exceeds duration_ticks=%d", m.LoopPoints.EndTick, g.DurationTicks))
	}
	if m.LoopPoints.StartTick > m.LoopPoints.EndTick {
		problems = append(problems, fmt.Sprintf("loop_points.start_tick=%d is after end_tick=%d", m.LoopPoints.StartTick, m.LoopPoints.EndTick))
	}

	validKinds := map[string]bool{"mocap": true, "human": true, "generative": true}
	if !validKinds[m.Authorship.Kind] {
		problems = append(problems, fmt.Sprintf("authorship.kind=%q is not one of mocap|human|generative", m.Authorship.Kind))
	}

	return problems
}
