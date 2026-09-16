// gmesh.go — Go-side writer for the .gmesh binary format, mirroring
// src/gmesh.c's layout exactly (see format/GMESH_FORMAT.md). Same
// write-only-from-Go rationale as gband.go/gskel.go.
package main

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
)

const gmeshHeaderSize = 16
const gmeshVertexRecordSize = 52

// GMeshVertex mirrors the C GMeshVertex struct field-for-field.
type GMeshVertex struct {
	Position    [3]float32
	Normal      [3]float32
	UV          [2]float32
	BoneIndices [4]uint8
	BoneWeights [4]float32
}

// GMeshFile is the in-memory representation of a .gmesh binary asset.
type GMeshFile struct {
	Version  uint32
	Vertices []GMeshVertex
	Indices  []uint32
}

// WriteFile writes m to path in the binary layout format/GMESH_FORMAT.md
// defines.
func (m *GMeshFile) WriteFile(path string) error {
	if len(m.Vertices) == 0 {
		return fmt.Errorf("gmesh: at least one vertex is required")
	}
	if len(m.Indices) == 0 || len(m.Indices)%3 != 0 {
		return fmt.Errorf("gmesh: index_count must be a nonzero multiple of 3, got %d", len(m.Indices))
	}

	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()

	header := make([]byte, gmeshHeaderSize)
	copy(header[0:4], "GMSH")
	binary.LittleEndian.PutUint32(header[4:8], m.Version)
	binary.LittleEndian.PutUint32(header[8:12], uint32(len(m.Vertices)))
	binary.LittleEndian.PutUint32(header[12:16], uint32(len(m.Indices)))
	if _, err := f.Write(header); err != nil {
		return err
	}

	for _, v := range m.Vertices {
		rec := make([]byte, gmeshVertexRecordSize)
		for k := 0; k < 3; k++ {
			binary.LittleEndian.PutUint32(rec[0+k*4:4+k*4], math.Float32bits(v.Position[k]))
		}
		for k := 0; k < 3; k++ {
			binary.LittleEndian.PutUint32(rec[12+k*4:16+k*4], math.Float32bits(v.Normal[k]))
		}
		for k := 0; k < 2; k++ {
			binary.LittleEndian.PutUint32(rec[24+k*4:28+k*4], math.Float32bits(v.UV[k]))
		}
		copy(rec[32:36], v.BoneIndices[:])
		for k := 0; k < 4; k++ {
			binary.LittleEndian.PutUint32(rec[36+k*4:40+k*4], math.Float32bits(v.BoneWeights[k]))
		}
		if _, err := f.Write(rec); err != nil {
			return err
		}
	}

	idxBuf := make([]byte, 4)
	for _, idx := range m.Indices {
		binary.LittleEndian.PutUint32(idxBuf, idx)
		if _, err := f.Write(idxBuf); err != nil {
			return err
		}
	}
	return nil
}
