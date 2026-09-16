// gskel.go — Go-side writer for the .gskel binary format, mirroring
// src/gskel.c's layout exactly (see format/GSKEL_FORMAT.md). Same
// no-cgo, write-only-from-Go rationale as gband.go's own header comment
// (the Go side only ever produces these files; src/gskel.c is what
// samples them at runtime).
package main

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
)

const (
	gskelHeaderSize      = 12
	gskelJointRecordSize = 128
	gskelNameLen         = 32
	gskelMaxJoints       = 64 // matches src/gskel.h's GSKEL_MAX_JOINTS exactly
)

// GSkelJoint mirrors the C GSkelJoint struct field-for-field.
type GSkelJoint struct {
	Name            string
	ParentIndex     int32 // -1 for root; must be < this joint's own index
	RestTranslation [3]float32
	RestRotation    [4]float32  // quaternion, x/y/z/w
	InverseBind     [16]float32 // column-major
}

// GSkelFile is the in-memory representation of a .gskel binary asset.
type GSkelFile struct {
	Version uint32
	Joints  []GSkelJoint
}

// WriteFile writes s to path in the binary layout format/GSKEL_FORMAT.md
// defines.
func (s *GSkelFile) WriteFile(path string) error {
	if len(s.Joints) == 0 {
		return fmt.Errorf("gskel: at least one joint is required")
	}
	if len(s.Joints) > gskelMaxJoints {
		return fmt.Errorf("gskel: %d joints exceeds GSKEL_MAX_JOINTS (%d)", len(s.Joints), gskelMaxJoints)
	}
	for i, j := range s.Joints {
		if j.ParentIndex >= int32(i) {
			return fmt.Errorf("gskel: joint %d (%q) has parent_index %d, must be < %d (parent-before-child ordering is required, not just conventional)", i, j.Name, j.ParentIndex, i)
		}
		if len(j.Name) >= gskelNameLen {
			return fmt.Errorf("gskel: joint %d name %q is %d bytes, must be < %d", i, j.Name, len(j.Name), gskelNameLen)
		}
	}

	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()

	header := make([]byte, gskelHeaderSize)
	copy(header[0:4], "GSKL")
	binary.LittleEndian.PutUint32(header[4:8], s.Version)
	binary.LittleEndian.PutUint32(header[8:12], uint32(len(s.Joints)))
	if _, err := f.Write(header); err != nil {
		return err
	}

	for _, j := range s.Joints {
		rec := make([]byte, gskelJointRecordSize)
		copy(rec[0:gskelNameLen], j.Name)
		binary.LittleEndian.PutUint32(rec[32:36], uint32(j.ParentIndex))
		for k := 0; k < 3; k++ {
			binary.LittleEndian.PutUint32(rec[36+k*4:40+k*4], math.Float32bits(j.RestTranslation[k]))
		}
		for k := 0; k < 4; k++ {
			binary.LittleEndian.PutUint32(rec[48+k*4:52+k*4], math.Float32bits(j.RestRotation[k]))
		}
		for k := 0; k < 16; k++ {
			binary.LittleEndian.PutUint32(rec[64+k*4:68+k*4], math.Float32bits(j.InverseBind[k]))
		}
		if _, err := f.Write(rec); err != nil {
			return err
		}
	}
	return nil
}
