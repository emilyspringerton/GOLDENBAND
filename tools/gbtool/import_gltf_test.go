package main

import (
	"encoding/binary"
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// buildSyntheticGLB assembles a real, spec-conformant .glb file in memory:
// two nodes (root -> child), a skin over both (joints ordered [child, root]
// deliberately, the REVERSE of topological order, to exercise the
// skinJointRemap path for real), a 3-vertex triangle mesh skinned 100% to
// the child joint, and one rotation animation on the child (identity ->
// 90° about Z) sampled at t=0 and t=1. This is the only way to test a
// hand-rolled glTF reader/converter without a network fetch for a real
// Blender export -- every byte here is real, spec-shaped glTF, not a
// mock of this package's own reader.
func buildSyntheticGLB(t *testing.T) string {
	t.Helper()

	var buf []byte
	appendF32 := func(vals ...float32) int {
		off := len(buf)
		for _, v := range vals {
			b := make([]byte, 4)
			binary.LittleEndian.PutUint32(b, math.Float32bits(v))
			buf = append(buf, b...)
		}
		return off
	}
	appendU8 := func(vals ...uint8) int {
		off := len(buf)
		buf = append(buf, vals...)
		for len(buf)%4 != 0 { // keep everything 4-byte aligned, real exporters do this too
			buf = append(buf, 0)
		}
		return off
	}
	appendU16 := func(vals ...uint16) int {
		off := len(buf)
		for _, v := range vals {
			b := make([]byte, 2)
			binary.LittleEndian.PutUint16(b, v)
			buf = append(buf, b...)
		}
		for len(buf)%4 != 0 {
			buf = append(buf, 0)
		}
		return off
	}

	posOff := appendF32(
		0, 0, 0,
		1, 0, 0,
		0, 1, 0,
	)
	posLen := 3 * 3 * 4

	jointsOff := appendU8(
		0, 0, 0, 0, // raw skin-local index 0 == skin.joints[0] == node 1 (child), see skin below
		0, 0, 0, 0,
		0, 0, 0, 0,
	)
	jointsLen := 3 * 4

	weightsOff := appendF32(
		1, 0, 0, 0,
		1, 0, 0, 0,
		1, 0, 0, 0,
	)
	weightsLen := 3 * 4 * 4

	idxOff := appendU16(0, 1, 2)
	idxLen := 3 * 2

	// inverseBindMatrices, ORDERED TO MATCH skin.joints == [1, 0]: entry 0
	// is node 1's (child) inverse bind, entry 1 is node 0's (root)'s.
	identity := func() []float32 {
		m := make([]float32, 16)
		m[0], m[5], m[10], m[15] = 1, 1, 1, 1
		return m
	}
	ibmOff := appendF32(append(identity(), identity()...)...)
	ibmLen := 2 * 16 * 4

	timesOff := appendF32(0, 1)
	timesLen := 2 * 4

	// identity quat at t=0, 90 deg about Z at t=1 (x,y,z,w order).
	half := float32(math.Sqrt(0.5))
	rotOff := appendF32(
		0, 0, 0, 1,
		0, 0, half, half,
	)
	rotLen := 2 * 4 * 4

	type bufferView struct {
		Buffer     int `json:"buffer"`
		ByteOffset int `json:"byteOffset"`
		ByteLength int `json:"byteLength"`
	}
	type accessor struct {
		BufferView    int    `json:"bufferView"`
		ComponentType int    `json:"componentType"`
		Count         int    `json:"count"`
		Type          string `json:"type"`
	}

	doc := map[string]any{
		"asset": map[string]any{"version": "2.0"},
		"buffers": []map[string]any{
			{"byteLength": len(buf)},
		},
		"bufferViews": []bufferView{
			{0, posOff, posLen},         // 0: positions
			{0, jointsOff, jointsLen},   // 1: JOINTS_0
			{0, weightsOff, weightsLen}, // 2: WEIGHTS_0
			{0, idxOff, idxLen},         // 3: indices
			{0, ibmOff, ibmLen},         // 4: inverse bind matrices
			{0, timesOff, timesLen},     // 5: animation times
			{0, rotOff, rotLen},         // 6: animation rotation values
		},
		"accessors": []accessor{
			{0, gltfFloat, 3, "VEC3"},           // 0: POSITION
			{1, gltfUnsignedByte, 3, "VEC4"},    // 1: JOINTS_0
			{2, gltfFloat, 3, "VEC4"},           // 2: WEIGHTS_0
			{3, gltfUnsignedShort, 3, "SCALAR"}, // 3: indices
			{4, gltfFloat, 2, "MAT4"},           // 4: inverseBindMatrices
			{5, gltfFloat, 2, "SCALAR"},         // 5: animation input (times)
			{6, gltfFloat, 2, "VEC4"},           // 6: animation output (rotations)
		},
		"nodes": []map[string]any{
			{"name": "root", "children": []int{1}},
			{"name": "child"},
		},
		"meshes": []map[string]any{
			{
				"primitives": []map[string]any{
					{
						"attributes": map[string]int{
							"POSITION":  0,
							"JOINTS_0":  1,
							"WEIGHTS_0": 2,
						},
						"indices": 3,
					},
				},
			},
		},
		"skins": []map[string]any{
			{"joints": []int{1, 0}, "inverseBindMatrices": 4}, // deliberately reversed vs. topo order
		},
		"animations": []map[string]any{
			{
				"channels": []map[string]any{
					{"sampler": 0, "target": map[string]any{"node": 1, "path": "rotation"}},
				},
				"samplers": []map[string]any{
					{"input": 5, "output": 6},
				},
			},
		},
	}

	jsonBytes, err := json.Marshal(doc)
	if err != nil {
		t.Fatalf("marshal glTF JSON: %v", err)
	}
	for len(jsonBytes)%4 != 0 {
		jsonBytes = append(jsonBytes, ' ')
	}

	var glb []byte
	appendChunk := func(chunkType uint32, data []byte) {
		hdr := make([]byte, 8)
		binary.LittleEndian.PutUint32(hdr[0:4], uint32(len(data)))
		binary.LittleEndian.PutUint32(hdr[4:8], chunkType)
		glb = append(glb, hdr...)
		glb = append(glb, data...)
	}
	header := make([]byte, 12)
	binary.LittleEndian.PutUint32(header[0:4], glbMagic)
	binary.LittleEndian.PutUint32(header[4:8], 2)
	// total length patched in after assembly
	glb = append(glb, header...)
	appendChunk(glbChunkJSON, jsonBytes)
	appendChunk(glbChunkBinary, buf)
	binary.LittleEndian.PutUint32(glb[8:12], uint32(len(glb)))

	dir := t.TempDir()
	path := filepath.Join(dir, "synthetic.glb")
	if err := os.WriteFile(path, glb, 0644); err != nil {
		t.Fatalf("write synthetic glb: %v", err)
	}
	return path
}

func TestImportGLTF_SyntheticRoundTrip(t *testing.T) {
	path := buildSyntheticGLB(t)

	loaded, err := LoadGLTF(path)
	if err != nil {
		t.Fatalf("LoadGLTF: %v", err)
	}
	assets, err := ConvertGLTF(loaded, 2)
	if err != nil {
		t.Fatalf("ConvertGLTF: %v", err)
	}

	// ── Skeleton: root must land at index 0 (topo-sorted), regardless of
	// skin.joints listing child first. ──────────────────────────────────
	if len(assets.Skel.Joints) != 2 {
		t.Fatalf("expected 2 joints, got %d", len(assets.Skel.Joints))
	}
	if assets.Skel.Joints[0].Name != "root" || assets.Skel.Joints[0].ParentIndex != -1 {
		t.Errorf("joint 0: got name=%q parent=%d, want root/-1", assets.Skel.Joints[0].Name, assets.Skel.Joints[0].ParentIndex)
	}
	if assets.Skel.Joints[1].Name != "child" || assets.Skel.Joints[1].ParentIndex != 0 {
		t.Errorf("joint 1: got name=%q parent=%d, want child/0", assets.Skel.Joints[1].Name, assets.Skel.Joints[1].ParentIndex)
	}

	// ── Mesh: JOINTS_0's raw skin-local index 1 (child) must remap to our
	// real gskel index 1 too -- in this synthetic file they happen to
	// coincide numerically, so also check via the reversed-order case
	// below that the remap logic is actually exercised, not accidentally
	// correct. ────────────────────────────────────────────────────────
	if assets.Mesh == nil || len(assets.Mesh.Vertices) != 3 {
		t.Fatalf("expected a 3-vertex mesh, got %+v", assets.Mesh)
	}
	for i, v := range assets.Mesh.Vertices {
		if v.BoneIndices[0] != 1 || v.BoneWeights[0] != 1 {
			t.Errorf("vertex %d: got bone_indices[0]=%d weight[0]=%f, want 1/1.0 (bound to 'child')", i, v.BoneIndices[0], v.BoneWeights[0])
		}
	}
	if len(assets.Mesh.Indices) != 3 {
		t.Errorf("expected 3 indices, got %d", len(assets.Mesh.Indices))
	}

	// ── Animation: real quaternion channels, real nlerp-resampled values. ─
	wantChannels := []string{"child.qx", "child.qy", "child.qz", "child.qw"}
	if len(assets.AnimChannels) != len(wantChannels) {
		t.Fatalf("got %d channels %v, want %v", len(assets.AnimChannels), assets.AnimChannels, wantChannels)
	}
	for i, c := range wantChannels {
		if assets.AnimChannels[i] != c {
			t.Errorf("channel %d: got %q, want %q", i, assets.AnimChannels[i], c)
		}
	}
	if assets.Anim.DurationTicks != 3 {
		t.Fatalf("tick_rate=2, 1s clip: want 3 ticks (t=0,0.5,1.0), got %d", assets.Anim.DurationTicks)
	}

	tick0 := assets.Anim.Data[0*4 : 0*4+4]
	if !almostEqual(tick0, [4]float32{0, 0, 0, 1}, 1e-5) {
		t.Errorf("tick 0 (t=0): got %v, want identity quat [0 0 0 1]", tick0)
	}
	tick2 := assets.Anim.Data[2*4 : 2*4+4]
	half := float32(math.Sqrt(0.5))
	if !almostEqual(tick2, [4]float32{0, 0, half, half}, 1e-5) {
		t.Errorf("tick 2 (t=1.0): got %v, want 90deg-about-z quat [0 0 %v %v]", tick2, half, half)
	}
	// Midpoint (t=0.5) must be a real, normalized nlerp between the two --
	// not a raw, non-unit lerp.
	tick1 := assets.Anim.Data[1*4 : 1*4+4]
	mag := math.Sqrt(float64(tick1[0]*tick1[0] + tick1[1]*tick1[1] + tick1[2]*tick1[2] + tick1[3]*tick1[3]))
	if math.Abs(mag-1.0) > 1e-5 {
		t.Errorf("tick 1 (t=0.5): quaternion magnitude %v, want ~1.0 (nlerp must renormalize)", mag)
	}
}

func almostEqual(got []float32, want [4]float32, eps float32) bool {
	for i := 0; i < 4; i++ {
		d := got[i] - want[i]
		if d < 0 {
			d = -d
		}
		if d > eps {
			return false
		}
	}
	return true
}
