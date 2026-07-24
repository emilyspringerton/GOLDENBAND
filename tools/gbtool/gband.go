// gband.go — Go-side reader/writer for the .gband binary format, mirroring
// src/gband.c's layout exactly (see format/GBAND_FORMAT.md). Kept
// independent of the C implementation (no cgo) since the Go side only
// ever needs to produce and inspect files, never sample them at runtime.
package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"os"
)

const gbandHeaderSize = 84

// GBandFile is the in-memory representation of a .gband binary asset.
type GBandFile struct {
	Version       uint32
	TickRate      uint32
	DurationTicks uint32
	NumChannels   uint32
	SkeletonHash  [32]byte
	ContentHash   [32]byte
	Data          []float32 // len == DurationTicks * NumChannels, row-major by tick
}

// ComputeContentHash returns sha256 over the raw channel-data bytes, the
// same computation src/gband.c's gb_verify performs.
func (g *GBandFile) ComputeContentHash() [32]byte {
	buf := new(bytes.Buffer)
	buf.Grow(len(g.Data) * 4)
	for _, v := range g.Data {
		binary.Write(buf, binary.LittleEndian, v) //nolint:errcheck
	}
	return sha256.Sum256(buf.Bytes())
}

// WriteFile writes g to path in the binary layout format/GBAND_FORMAT.md
// defines. ContentHash is (re)computed from Data before writing, so
// callers never need to set it themselves.
func (g *GBandFile) WriteFile(path string) error {
	g.ContentHash = g.ComputeContentHash()

	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()

	header := make([]byte, gbandHeaderSize)
	copy(header[0:4], "GBND")
	binary.LittleEndian.PutUint32(header[4:8], g.Version)
	binary.LittleEndian.PutUint32(header[8:12], g.TickRate)
	binary.LittleEndian.PutUint32(header[12:16], g.DurationTicks)
	binary.LittleEndian.PutUint32(header[16:20], g.NumChannels)
	copy(header[20:52], g.SkeletonHash[:])
	copy(header[52:84], g.ContentHash[:])

	if _, err := f.Write(header); err != nil {
		return err
	}
	for _, v := range g.Data {
		if err := binary.Write(f, binary.LittleEndian, v); err != nil {
			return err
		}
	}
	return nil
}

// ReadGBandFile reads and structurally validates a .gband binary file.
func ReadGBandFile(path string) (*GBandFile, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	if len(raw) < gbandHeaderSize {
		return nil, fmt.Errorf("%s: too short to be a .gband file (%d bytes)", path, len(raw))
	}
	if string(raw[0:4]) != "GBND" {
		return nil, fmt.Errorf("%s: bad magic %q, expected GBND", path, raw[0:4])
	}

	g := &GBandFile{
		Version:       binary.LittleEndian.Uint32(raw[4:8]),
		TickRate:      binary.LittleEndian.Uint32(raw[8:12]),
		DurationTicks: binary.LittleEndian.Uint32(raw[12:16]),
		NumChannels:   binary.LittleEndian.Uint32(raw[16:20]),
	}
	copy(g.SkeletonHash[:], raw[20:52])
	copy(g.ContentHash[:], raw[52:84])

	wantFloats := int(g.DurationTicks) * int(g.NumChannels)
	wantBytes := gbandHeaderSize + wantFloats*4
	if len(raw) != wantBytes {
		return nil, fmt.Errorf("%s: size mismatch — header declares %d ticks x %d channels (expect %d bytes), file is %d bytes",
			path, g.DurationTicks, g.NumChannels, wantBytes, len(raw))
	}

	g.Data = make([]float32, wantFloats)
	r := bytes.NewReader(raw[gbandHeaderSize:])
	if err := binary.Read(r, binary.LittleEndian, g.Data); err != nil {
		return nil, fmt.Errorf("%s: reading channel data: %w", path, err)
	}
	return g, nil
}

// Verify reports whether the file's stored ContentHash matches a
// recomputed hash over its own Data.
func (g *GBandFile) Verify() bool {
	return g.ComputeContentHash() == g.ContentHash
}
