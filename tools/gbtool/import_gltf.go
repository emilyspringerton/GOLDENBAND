// import_gltf.go — converts a parsed glTF document (gltf.go) into GOLDEN
// BAND's own three real asset types: .gskel (skeleton), .gmesh (geometry),
// and .gband (animation, with real quaternion rotation channels — see
// format/GBAND_FORMAT.md's "<joint>.qx/.qy/.qz/.qw" convention). Founder
// real-time direction: "let's start iterating towards nock tools modeler
// (blender) and golden band we need to be able to import quaternion
// animations into nock golden band" -> "quaternion models too, potentially"
// -> scoped to glTF import including meshes/skeletons. glTF is Blender's
// own native quaternion-based export format (its "glTF Binary (.glb)" /
// "glTF Separate" exporters), so this is the real, direct unlock: no
// Blender plugin needed for v0, just "File > Export > glTF 2.0" into this
// importer.
//
// v0 scope, documented rather than silently assumed:
//   - First skin, first mesh's first primitive, first animation only.
//     Multi-skeleton/multi-mesh/multi-clip glTF files need re-running
//     with a --node/--animation selector this pass doesn't add yet.
//   - LINEAR sampler interpolation only (STEP/CUBICSPLINE unsupported —
//     same "documented gap, not silently wrong" posture BVH import and
//     the format docs already take for their own gaps).
//   - Rotation channels are resampled via per-component lerp + renormalize
//     (nlerp), not true slerp. Honest approximation for adjacent glTF
//     keyframes (small angular deltas, the common case for a Blender-
//     authored clip) — flagged here and in the format doc, not silently
//     assumed correct for widely-spaced keyframes.
//   - A mesh with no skin (JOINTS_0/WEIGHTS_0 accessors absent) still
//     imports: every vertex binds 100% to joint 0 against a synthesized
//     single-root identity skeleton, so an unrigged static mesh export
//     still produces a valid, pairable .gmesh/.gskel rather than failing.
package main

import (
	"fmt"
	"math"
	"sort"
)

// gltfJointInfo is one joint's own real, resolved data before it gets
// placed into topological (parent-before-child) order.
type gltfJointInfo struct {
	nodeIndex   int
	name        string
	parentNode  int // -1 if this joint's parent isn't itself a joint (or is the scene root)
	translation [3]float32
	rotation    [4]float32 // x,y,z,w
}

// ImportedGLTFAssets is everything an import --gltf run produces.
type ImportedGLTFAssets struct {
	Skel         *GSkelFile
	Mesh         *GMeshFile // nil if the glTF has no meshes at all
	Anim         *GBandFile
	AnimChannels []string
	TickRate     uint32
}

// ConvertGLTF walks a loaded glTF document and produces the real GOLDEN
// BAND assets described above. tickRate governs animation resampling
// only (skeleton/mesh data has no time axis).
func ConvertGLTF(g *LoadedGLTF, tickRate uint32) (*ImportedGLTFAssets, error) {
	doc := &g.Doc

	// ── Parent map over ALL nodes (needed to find each joint's nearest
	// joint-ancestor, even when non-joint nodes sit between them in the
	// real scene hierarchy). ────────────────────────────────────────────
	nodeParent := make([]int, len(doc.Nodes))
	for i := range nodeParent {
		nodeParent[i] = -1
	}
	for ni, n := range doc.Nodes {
		for _, c := range n.Children {
			if c >= 0 && c < len(nodeParent) {
				nodeParent[c] = ni
			}
		}
	}

	var skel *GSkelFile
	var nodeToJointIdx map[int]int // node index -> our own, final gskel joint index
	var skinJointRemap []int       // JOINTS_0's own raw per-vertex index -> our final gskel joint index

	if len(doc.Skins) > 0 {
		var err error
		skel, nodeToJointIdx, err = buildSkeleton(doc, g, nodeParent, doc.Skins[0])
		if err != nil {
			return nil, err
		}
		// JOINTS_0 stores small integers that index into THIS skin's own
		// joints[] array (glTF's own numbering, e.g. 0,1,2...), not node
		// indices directly -- skinJointRemap[raw] resolves that through
		// nodeToJointIdx in one step so buildMesh never has to know about
		// skins at all.
		skinJointRemap = make([]int, len(doc.Skins[0].Joints))
		for raw, nodeIdx := range doc.Skins[0].Joints {
			skinJointRemap[raw] = nodeToJointIdx[nodeIdx]
		}
	} else {
		// No skin at all in the file -- synthesize a trivial single-root
		// identity skeleton so a plain, unrigged mesh export still has
		// something real to pair with (see this file's own header note).
		skel = &GSkelFile{
			Version: 1,
			Joints: []GSkelJoint{{
				Name:         "root",
				ParentIndex:  -1,
				RestRotation: [4]float32{0, 0, 0, 1},
				InverseBind:  identityMat4(),
			}},
		}
		nodeToJointIdx = map[int]int{}
	}

	var mesh *GMeshFile
	if len(doc.Meshes) > 0 {
		var err error
		mesh, err = buildMesh(doc, g, doc.Meshes[0], skinJointRemap)
		if err != nil {
			return nil, err
		}
	}

	var anim *GBandFile
	var channels []string
	if len(doc.Animations) > 0 {
		var err error
		anim, channels, err = buildAnimation(doc, g, doc.Animations[0], tickRate)
		if err != nil {
			return nil, err
		}
	}

	return &ImportedGLTFAssets{Skel: skel, Mesh: mesh, Anim: anim, AnimChannels: channels, TickRate: tickRate}, nil
}

func identityMat4() [16]float32 {
	var m [16]float32
	m[0], m[5], m[10], m[15] = 1, 1, 1, 1
	return m
}

// buildSkeleton resolves skin.Joints (glTF node indices, unordered w.r.t.
// parent/child) into a real GSkelFile whose joints[] is topologically
// sorted (parent_index always < self index — .gskel's own hard
// requirement, not just a convention: see format/GSKEL_FORMAT.md).
func buildSkeleton(doc *gltfDoc, g *LoadedGLTF, nodeParent []int, skin gltfSkin) (*GSkelFile, map[int]int, error) {
	if len(skin.Joints) == 0 {
		return nil, nil, fmt.Errorf("gltf: skin has zero joints")
	}
	if len(skin.Joints) > gskelMaxJoints {
		return nil, nil, fmt.Errorf("gltf: skin has %d joints, exceeds GSKEL_MAX_JOINTS (%d)", len(skin.Joints), gskelMaxJoints)
	}

	isJointNode := make(map[int]bool, len(skin.Joints))
	for _, ni := range skin.Joints {
		isJointNode[ni] = true
	}

	// nearestJointAncestor walks up the real node hierarchy until it finds
	// another joint node (or runs out of ancestors).
	nearestJointAncestor := func(ni int) int {
		p := nodeParent[ni]
		for p != -1 {
			if isJointNode[p] {
				return p
			}
			p = nodeParent[p]
		}
		return -1
	}

	infos := make(map[int]*gltfJointInfo, len(skin.Joints))
	for _, ni := range skin.Joints {
		n := doc.Nodes[ni]
		info := &gltfJointInfo{
			nodeIndex:  ni,
			name:       n.Name,
			parentNode: nearestJointAncestor(ni),
		}
		if info.name == "" {
			info.name = fmt.Sprintf("joint_%d", ni)
		}
		copy3(&info.translation, n.Translation, [3]float64{0, 0, 0})
		copyQuat(&info.rotation, n.Rotation)
		infos[ni] = info
	}

	// Topological order: roots first (parentNode == -1), then children of
	// already-placed joints, breadth-first — guarantees parent_index <
	// self index in the final array, exactly what .gskel requires.
	var order []int
	placed := make(map[int]bool, len(skin.Joints))
	childrenOf := make(map[int][]int)
	var roots []int
	for _, ni := range skin.Joints {
		p := infos[ni].parentNode
		if p == -1 {
			roots = append(roots, ni)
		} else {
			childrenOf[p] = append(childrenOf[p], ni)
		}
	}
	sort.Ints(roots)
	queue := append([]int{}, roots...)
	for len(queue) > 0 {
		ni := queue[0]
		queue = queue[1:]
		if placed[ni] {
			continue
		}
		placed[ni] = true
		order = append(order, ni)
		kids := append([]int{}, childrenOf[ni]...)
		sort.Ints(kids)
		queue = append(queue, kids...)
	}
	if len(order) != len(skin.Joints) {
		return nil, nil, fmt.Errorf("gltf: skin joint hierarchy is not a forest reachable from its own roots (got %d of %d joints)", len(order), len(skin.Joints))
	}

	nodeToJointIdx := make(map[int]int, len(order))
	for idx, ni := range order {
		nodeToJointIdx[ni] = idx
	}

	var ibms [][]float64
	if skin.InverseBindMatrices != nil {
		var err error
		ibms, err = g.accessorFloats(*skin.InverseBindMatrices)
		if err != nil {
			return nil, nil, fmt.Errorf("gltf: skin inverseBindMatrices: %w", err)
		}
		if len(ibms) != len(skin.Joints) {
			return nil, nil, fmt.Errorf("gltf: skin has %d joints but %d inverse bind matrices", len(skin.Joints), len(ibms))
		}
	}

	joints := make([]GSkelJoint, len(order))
	for idx, ni := range order {
		info := infos[ni]
		gj := GSkelJoint{
			Name:            info.name,
			RestTranslation: info.translation,
			RestRotation:    info.rotation,
			InverseBind:     identityMat4(),
		}
		if info.parentNode == -1 {
			gj.ParentIndex = -1
		} else {
			gj.ParentIndex = int32(nodeToJointIdx[info.parentNode])
		}
		if ibms != nil {
			// skin.Joints[origPos] pairs with ibms[origPos] per the glTF
			// spec's own required ordering -- find origPos for this node.
			for origPos, jn := range skin.Joints {
				if jn == ni {
					for k := 0; k < 16 && k < len(ibms[origPos]); k++ {
						gj.InverseBind[k] = float32(ibms[origPos][k])
					}
					break
				}
			}
		}
		joints[idx] = gj
	}

	return &GSkelFile{Version: 1, Joints: joints}, nodeToJointIdx, nil
}

func copy3(dst *[3]float32, src []float64, def [3]float64) {
	for k := 0; k < 3; k++ {
		v := def[k]
		if k < len(src) {
			v = src[k]
		}
		dst[k] = float32(v)
	}
}

func copyQuat(dst *[4]float32, src []float64) {
	def := [4]float64{0, 0, 0, 1}
	for k := 0; k < 4; k++ {
		v := def[k]
		if k < len(src) {
			v = src[k]
		}
		dst[k] = float32(v)
	}
}

// buildMesh converts the first primitive of mesh into a real GMeshFile.
// nodeToJointIdx remaps glTF's own per-primitive JOINTS_0 values (indices
// into the OWNING NODE's skin.joints array) into our final, topologically
// sorted .gskel joint indices.
func buildMesh(doc *gltfDoc, g *LoadedGLTF, mesh gltfMesh, skinJointRemap []int) (*GMeshFile, error) {
	if len(mesh.Primitives) == 0 {
		return nil, fmt.Errorf("gltf: mesh has zero primitives")
	}
	prim := mesh.Primitives[0]

	posIdx, ok := prim.Attributes["POSITION"]
	if !ok {
		return nil, fmt.Errorf("gltf: mesh primitive has no POSITION attribute")
	}
	positions, err := g.accessorFloats(posIdx)
	if err != nil {
		return nil, fmt.Errorf("gltf: POSITION: %w", err)
	}
	vertexCount := len(positions)

	normals := make([][]float64, vertexCount)
	if ni, ok := prim.Attributes["NORMAL"]; ok {
		normals, err = g.accessorFloats(ni)
		if err != nil {
			return nil, fmt.Errorf("gltf: NORMAL: %w", err)
		}
	} else {
		for i := range normals {
			normals[i] = []float64{0, 1, 0} // documented default -- no NORMAL attribute exported
		}
	}

	uvs := make([][]float64, vertexCount)
	if ui, ok := prim.Attributes["TEXCOORD_0"]; ok {
		uvs, err = g.accessorFloats(ui)
		if err != nil {
			return nil, fmt.Errorf("gltf: TEXCOORD_0: %w", err)
		}
	} else {
		for i := range uvs {
			uvs[i] = []float64{0, 0}
		}
	}

	joints := make([][]float64, vertexCount)
	weights := make([][]float64, vertexCount)
	if ji, jok := prim.Attributes["JOINTS_0"]; jok {
		wi, wok := prim.Attributes["WEIGHTS_0"]
		if !wok {
			return nil, fmt.Errorf("gltf: mesh has JOINTS_0 but no WEIGHTS_0")
		}
		joints, err = g.accessorFloats(ji)
		if err != nil {
			return nil, fmt.Errorf("gltf: JOINTS_0: %w", err)
		}
		weights, err = g.accessorFloats(wi)
		if err != nil {
			return nil, fmt.Errorf("gltf: WEIGHTS_0: %w", err)
		}
	} else {
		// Unrigged mesh -- bind everything to joint 0, weight 1.0 (see
		// this file's own header note on the synthesized identity skeleton).
		for i := 0; i < vertexCount; i++ {
			joints[i] = []float64{0, 0, 0, 0}
			weights[i] = []float64{1, 0, 0, 0}
		}
	}

	verts := make([]GMeshVertex, vertexCount)
	for i := 0; i < vertexCount; i++ {
		v := GMeshVertex{}
		copy3(&v.Position, positions[i], [3]float64{0, 0, 0})
		copy3(&v.Normal, normals[i], [3]float64{0, 1, 0})
		for k := 0; k < 2 && k < len(uvs[i]); k++ {
			v.UV[k] = float32(uvs[i][k])
		}
		for k := 0; k < 4 && k < len(joints[i]); k++ {
			raw := int(joints[i][k])
			final := raw
			if raw >= 0 && raw < len(skinJointRemap) {
				final = skinJointRemap[raw]
			}
			v.BoneIndices[k] = uint8(final)
		}
		for k := 0; k < 4 && k < len(weights[i]); k++ {
			v.BoneWeights[k] = float32(weights[i][k])
		}
		verts[i] = v
	}

	var indices []uint32
	if prim.Indices != nil {
		rows, err := g.accessorFloats(*prim.Indices)
		if err != nil {
			return nil, fmt.Errorf("gltf: primitive indices: %w", err)
		}
		indices = make([]uint32, len(rows))
		for i, r := range rows {
			indices[i] = uint32(r[0])
		}
	} else {
		// No index buffer -- triangle list is implicitly 0,1,2,3,4,5,...
		indices = make([]uint32, vertexCount-(vertexCount%3))
		for i := range indices {
			indices[i] = uint32(i)
		}
	}

	return &GMeshFile{Version: 1, Vertices: verts, Indices: indices}, nil
}

// buildAnimation resamples every LINEAR rotation/translation channel in
// anim to a uniform tick_rate, producing real quaternion (qx/qy/qz/qw)
// and translation (tx/ty/tz) channels named "<node-or-joint-name>.<comp>".
func buildAnimation(doc *gltfDoc, g *LoadedGLTF, anim gltfAnimation, tickRate uint32) (*GBandFile, []string, error) {
	// Group by (node, path) first so a quaternion's 4 components stay one
	// resampling unit (nlerp needs all 4 together, not resampled
	// independently per axis).
	type groupKey struct {
		node int
		path string
	}
	groups := map[groupKey]gltfAnimSampler{}
	for _, ch := range anim.Channels {
		if ch.Target.Node == nil {
			continue // targeting a mesh weight/whole-scene channel -- not supported (v0 gap)
		}
		if ch.Target.Path != "translation" && ch.Target.Path != "rotation" {
			continue // scale/weights -- documented v0 gap
		}
		if ch.Sampler < 0 || ch.Sampler >= len(anim.Samplers) {
			continue
		}
		groups[groupKey{*ch.Target.Node, ch.Target.Path}] = anim.Samplers[ch.Sampler]
	}
	if len(groups) == 0 {
		return nil, nil, fmt.Errorf("gltf: animation %q has no supported (translation/rotation) channels", anim.Name)
	}

	maxTime := 0.0
	type decodedGroup struct {
		key    groupKey
		times  []float64
		values [][]float64 // len(times) rows, each len 3 (translation) or 4 (rotation)
	}
	var decoded []decodedGroup
	for key, sampler := range groups {
		timeRows, err := g.accessorFloats(sampler.Input)
		if err != nil {
			return nil, nil, fmt.Errorf("gltf: animation sampler input: %w", err)
		}
		valueRows, err := g.accessorFloats(sampler.Output)
		if err != nil {
			return nil, nil, fmt.Errorf("gltf: animation sampler output: %w", err)
		}
		if len(timeRows) != len(valueRows) {
			return nil, nil, fmt.Errorf("gltf: animation sampler has %d times but %d values", len(timeRows), len(valueRows))
		}
		times := make([]float64, len(timeRows))
		for i, r := range timeRows {
			times[i] = r[0]
			if r[0] > maxTime {
				maxTime = r[0]
			}
		}
		decoded = append(decoded, decodedGroup{key: key, times: times, values: valueRows})
	}
	if tickRate == 0 {
		return nil, nil, fmt.Errorf("gltf: tick rate must be > 0")
	}

	durationTicks := uint32(math.Floor(maxTime*float64(tickRate))) + 1
	if durationTicks < 1 {
		durationTicks = 1
	}

	var channelNames []string
	var perTick [][]float32 // [channel][tick]
	for _, dg := range decoded {
		n := doc.Nodes[dg.key.node]
		name := n.Name
		if name == "" {
			name = fmt.Sprintf("joint_%d", dg.key.node)
		}
		if dg.key.path == "rotation" {
			samples := resampleQuat(dg.times, dg.values, tickRate, durationTicks)
			for c, suffix := range []string{"qx", "qy", "qz", "qw"} {
				channelNames = append(channelNames, name+"."+suffix)
				col := make([]float32, durationTicks)
				for t := range col {
					col[t] = float32(samples[t][c])
				}
				perTick = append(perTick, col)
			}
		} else {
			samples := resampleLinear(dg.times, dg.values, tickRate, durationTicks, 3)
			for c, suffix := range []string{"tx", "ty", "tz"} {
				channelNames = append(channelNames, name+"."+suffix)
				col := make([]float32, durationTicks)
				for t := range col {
					col[t] = float32(samples[t][c])
				}
				perTick = append(perTick, col)
			}
		}
	}

	numChannels := len(channelNames)
	data := make([]float32, int(durationTicks)*numChannels)
	for t := 0; t < int(durationTicks); t++ {
		for c := 0; c < numChannels; c++ {
			data[t*numChannels+c] = perTick[c][t]
		}
	}

	return &GBandFile{
		Version:       1,
		TickRate:      tickRate,
		DurationTicks: durationTicks,
		NumChannels:   uint32(numChannels),
		Data:          data,
	}, channelNames, nil
}

// resampleLinear resamples a keyframed curve (times/values, values[i] has
// `comps` components) to durationTicks uniform samples at tickRate,
// clamping to the first/last keyframe outside the curve's own real range.
func resampleLinear(times []float64, values [][]float64, tickRate uint32, durationTicks uint32, comps int) [][]float64 {
	out := make([][]float64, durationTicks)
	for t := uint32(0); t < durationTicks; t++ {
		time := float64(t) / float64(tickRate)
		out[t] = sampleKeyframesAt(times, values, time, comps)
	}
	return out
}

// resampleQuat is resampleLinear's rotation counterpart: per-component
// lerp between the surrounding keyframes, then renormalized to a unit
// quaternion (nlerp) — see this file's own header note on why this isn't
// true slerp.
func resampleQuat(times []float64, values [][]float64, tickRate uint32, durationTicks uint32) [][]float64 {
	lin := resampleLinear(times, values, tickRate, durationTicks, 4)
	for i, q := range lin {
		mag := math.Sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
		if mag > 1e-12 {
			lin[i] = []float64{q[0] / mag, q[1] / mag, q[2] / mag, q[3] / mag}
		} else {
			lin[i] = []float64{0, 0, 0, 1}
		}
	}
	return lin
}

func sampleKeyframesAt(times []float64, values [][]float64, time float64, comps int) []float64 {
	n := len(times)
	if n == 0 {
		return make([]float64, comps)
	}
	if time <= times[0] {
		return append([]float64(nil), values[0]...)
	}
	if time >= times[n-1] {
		return append([]float64(nil), values[n-1]...)
	}
	// Linear scan -- real glTF animation clips from Blender have at most a
	// few hundred keyframes, well within "simple beats clever" territory
	// for a v0 import tool.
	for i := 0; i < n-1; i++ {
		if time >= times[i] && time <= times[i+1] {
			span := times[i+1] - times[i]
			w := 0.0
			if span > 1e-12 {
				w = (time - times[i]) / span
			}
			out := make([]float64, comps)
			for c := 0; c < comps; c++ {
				a, b := 0.0, 0.0
				if c < len(values[i]) {
					a = values[i][c]
				}
				if c < len(values[i+1]) {
					b = values[i+1][c]
				}
				out[c] = a + (b-a)*w
			}
			return out
		}
	}
	return append([]float64(nil), values[n-1]...)
}
