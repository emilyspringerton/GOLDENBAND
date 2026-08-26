package main

import (
	"encoding/hex"
	"os"
	"path/filepath"
	"testing"
)

func TestGBandFile_WriteReadRoundTrip(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "test.gband")

	g := &GBandFile{
		Version:       1,
		TickRate:      64,
		DurationTicks: 3,
		NumChannels:   2,
		Data:          []float32{0, 0, 1, 10, 2, 20},
	}
	if err := g.WriteFile(path); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	got, err := ReadGBandFile(path)
	if err != nil {
		t.Fatalf("ReadGBandFile: %v", err)
	}
	if got.TickRate != 64 || got.DurationTicks != 3 || got.NumChannels != 2 {
		t.Fatalf("header mismatch: %+v", got)
	}
	for i, v := range g.Data {
		if got.Data[i] != v {
			t.Errorf("Data[%d] = %v, want %v", i, got.Data[i], v)
		}
	}
	if !got.Verify() {
		t.Error("expected Verify() to pass on a freshly-written, unmodified file")
	}
}

func TestGBandFile_VerifyDetectsTamperedData(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "test.gband")

	g := &GBandFile{Version: 1, TickRate: 30, DurationTicks: 2, NumChannels: 1, Data: []float32{1, 2}}
	if err := g.WriteFile(path); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}
	got, err := ReadGBandFile(path)
	if err != nil {
		t.Fatalf("ReadGBandFile: %v", err)
	}
	got.Data[0] = 999
	if got.Verify() {
		t.Error("expected Verify() to fail after tampering with Data in memory")
	}
}

func TestReadGBandFile_RejectsBadMagic(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "bad.gband")
	writeRaw(t, path, []byte("XXXXrestofbytes"))

	_, err := ReadGBandFile(path)
	if err == nil {
		t.Fatal("expected an error for a file with bad magic")
	}
}

func TestReadGBandFile_RejectsSizeMismatch(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "short.gband")

	g := &GBandFile{Version: 1, TickRate: 30, DurationTicks: 5, NumChannels: 2, Data: []float32{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}}
	if err := g.WriteFile(path); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}
	// Now lie about duration_ticks in the header only, without rewriting the data.
	raw := readRaw(t, path)
	raw[12] = 99 // corrupt duration_ticks field
	writeRaw(t, path, raw)

	_, err := ReadGBandFile(path)
	if err == nil {
		t.Fatal("expected an error for a header/data size mismatch")
	}
}

func TestManifest_ValidateAgainst_Clean(t *testing.T) {
	dir := t.TempDir()
	binPath := filepath.Join(dir, "clip.gband")

	g := &GBandFile{Version: 1, TickRate: 60, DurationTicks: 2, NumChannels: 2, Data: []float32{0, 0, 1, 1}}
	if err := g.WriteFile(binPath); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}
	got, err := ReadGBandFile(binPath)
	if err != nil {
		t.Fatalf("ReadGBandFile: %v", err)
	}

	m := &Manifest{
		TickRate:      60,
		DurationTicks: 2,
		Channels:      []string{"a", "b"},
		ContentHash:   hex.EncodeToString(got.ContentHash[:]),
		Authorship:    Authorship{Kind: "mocap", Who: "test fixture"},
		LoopPoints:    LoopPoints{StartTick: 0, EndTick: 2},
	}
	if problems := m.ValidateAgainst(got); len(problems) != 0 {
		t.Errorf("expected no problems, got: %v", problems)
	}
}

func TestManifest_ValidateAgainst_CatchesMismatches(t *testing.T) {
	dir := t.TempDir()
	binPath := filepath.Join(dir, "clip.gband")

	g := &GBandFile{Version: 1, TickRate: 60, DurationTicks: 2, NumChannels: 2, Data: []float32{0, 0, 1, 1}}
	if err := g.WriteFile(binPath); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}
	got, err := ReadGBandFile(binPath)
	if err != nil {
		t.Fatalf("ReadGBandFile: %v", err)
	}

	m := &Manifest{
		TickRate:      30, // wrong
		DurationTicks: 2,
		Channels:      []string{"a"},                        // wrong count
		ContentHash:   "deadbeef",                           // wrong
		Authorship:    Authorship{Kind: "bogus"},            // invalid enum
		LoopPoints:    LoopPoints{StartTick: 5, EndTick: 1}, // start after end, and end > duration
	}
	problems := m.ValidateAgainst(got)
	if len(problems) < 5 {
		t.Errorf("expected at least 5 problems (tick_rate, channel count, content_hash, authorship kind, loop points), got %d: %v", len(problems), problems)
	}
}

func writeRaw(t *testing.T, path string, data []byte) {
	t.Helper()
	if err := os.WriteFile(path, data, 0644); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
}

func readRaw(t *testing.T, path string) []byte {
	t.Helper()
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read %s: %v", path, err)
	}
	return b
}

func TestSmoothstepCurve(t *testing.T) {
	curve := smoothstepCurve(16)
	if len(curve) != 16 {
		t.Fatalf("len = %d, want 16", len(curve))
	}
	if curve[0] != 0 {
		t.Errorf("curve[0] = %v, want 0 (ease-in starts at rest)", curve[0])
	}
	if curve[len(curve)-1] != 1 {
		t.Errorf("curve[last] = %v, want 1 (ease-out ends at rest)", curve[len(curve)-1])
	}
	// Monotonic, no overshoot -- every real ease-in/ease-out curve must be.
	for i := 1; i < len(curve); i++ {
		if curve[i] < curve[i-1] {
			t.Fatalf("curve not monotonic at index %d: %v -> %v", i, curve[i-1], curve[i])
		}
		if curve[i] < 0 || curve[i] > 1 {
			t.Fatalf("curve[%d] = %v out of [0,1]", i, curve[i])
		}
	}
	// Smoothstep's own signature shape: slower than linear near both ends
	// (zero velocity at the boundary), faster than linear in the middle.
	if curve[1] >= 1.0/15.0 {
		t.Errorf("curve[1] = %v, want < linear step %v (ease-in should start slow)", curve[1], 1.0/15.0)
	}
}

func TestSmoothstepCurve_SingleTick(t *testing.T) {
	curve := smoothstepCurve(1)
	if len(curve) != 1 || curve[0] != 1 {
		t.Fatalf("smoothstepCurve(1) = %v, want [1]", curve)
	}
}
