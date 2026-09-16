// gltf.go — a minimal glTF 2.0 reader: just enough of the spec to pull
// nodes/skins/meshes/animations out of a file Blender's own "glTF Binary
// (.glb)" or "glTF Separate (.gltf + .bin)" exporter produced. Not a
// general-purpose glTF library -- no extensions, no sparse accessors, no
// morph targets, no materials/textures, no interleaved-buffer edge cases
// beyond what a straightforward Blender export emits. Standard library
// only, matching this repo's "no engine dependency, no vendored deps"
// convention (see gband.go's own header comment).
package main

import (
	"encoding/base64"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
)

type gltfDoc struct {
	Buffers     []gltfBuffer     `json:"buffers"`
	BufferViews []gltfBufferView `json:"bufferViews"`
	Accessors   []gltfAccessor   `json:"accessors"`
	Nodes       []gltfNode       `json:"nodes"`
	Meshes      []gltfMesh       `json:"meshes"`
	Skins       []gltfSkin       `json:"skins"`
	Animations  []gltfAnimation  `json:"animations"`
}

type gltfBuffer struct {
	URI        string `json:"uri"`
	ByteLength int    `json:"byteLength"`
}

type gltfBufferView struct {
	Buffer     int `json:"buffer"`
	ByteOffset int `json:"byteOffset"`
	ByteLength int `json:"byteLength"`
	ByteStride int `json:"byteStride"`
}

type gltfAccessor struct {
	BufferView    *int   `json:"bufferView"`
	ByteOffset    int    `json:"byteOffset"`
	ComponentType int    `json:"componentType"`
	Count         int    `json:"count"`
	Type          string `json:"type"` // SCALAR|VEC2|VEC3|VEC4|MAT4
}

type gltfNode struct {
	Name        string    `json:"name"`
	Children    []int     `json:"children"`
	Mesh        *int      `json:"mesh"`
	Skin        *int      `json:"skin"`
	Translation []float64 `json:"translation"`
	Rotation    []float64 `json:"rotation"` // quaternion x,y,z,w
}

type gltfMesh struct {
	Primitives []gltfPrimitive `json:"primitives"`
}

type gltfPrimitive struct {
	Attributes map[string]int `json:"attributes"`
	Indices    *int           `json:"indices"`
}

type gltfSkin struct {
	InverseBindMatrices *int  `json:"inverseBindMatrices"`
	Joints              []int `json:"joints"`
}

type gltfAnimation struct {
	Name     string            `json:"name"`
	Channels []gltfAnimChannel `json:"channels"`
	Samplers []gltfAnimSampler `json:"samplers"`
}

type gltfAnimChannel struct {
	Sampler int            `json:"sampler"`
	Target  gltfAnimTarget `json:"target"`
}

type gltfAnimTarget struct {
	Node *int   `json:"node"`
	Path string `json:"path"` // "translation" | "rotation" | "scale" | "weights"
}

type gltfAnimSampler struct {
	Input  int `json:"input"`
	Output int `json:"output"`
}

// component type constants (the real glTF spec values, not made up).
const (
	gltfByte          = 5120
	gltfUnsignedByte  = 5121
	gltfShort         = 5122
	gltfUnsignedShort = 5123
	gltfUnsignedInt   = 5125
	gltfFloat         = 5126
)

var gltfTypeComponents = map[string]int{
	"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16,
}

// LoadedGLTF is a parsed document plus its resolved buffer bytes, ready
// for accessor decoding.
type LoadedGLTF struct {
	Doc     gltfDoc
	Buffers [][]byte // one entry per doc.Buffers, fully resolved
}

const (
	glbMagic       = 0x46546C67 // "glTF"
	glbChunkJSON   = 0x4E4F534A // "JSON"
	glbChunkBinary = 0x004E4942 // "BIN\0"
)

// LoadGLTF reads a .glb (binary container) or .gltf (JSON, with external
// or base64-embedded buffers) file and resolves every referenced buffer
// into memory.
func LoadGLTF(path string) (*LoadedGLTF, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}

	var jsonBytes []byte
	var glbBin []byte

	if len(raw) >= 12 && binary.LittleEndian.Uint32(raw[0:4]) == glbMagic {
		jsonBytes, glbBin, err = parseGLBContainer(raw)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", path, err)
		}
	} else {
		jsonBytes = raw
	}

	var doc gltfDoc
	if err := json.Unmarshal(jsonBytes, &doc); err != nil {
		return nil, fmt.Errorf("%s: parsing glTF JSON: %w", path, err)
	}

	buffers := make([][]byte, len(doc.Buffers))
	for i, b := range doc.Buffers {
		switch {
		case b.URI == "" && glbBin != nil:
			// The GLB spec's own convention: a buffer with no uri means
			// "the single embedded BIN chunk" -- only valid for buffer 0.
			buffers[i] = glbBin
		case strings.HasPrefix(b.URI, "data:"):
			decoded, err := decodeDataURI(b.URI)
			if err != nil {
				return nil, fmt.Errorf("%s: buffer %d: %w", path, i, err)
			}
			buffers[i] = decoded
		case b.URI != "":
			p := filepath.Join(filepath.Dir(path), b.URI)
			data, err := os.ReadFile(p)
			if err != nil {
				return nil, fmt.Errorf("%s: buffer %d (%s): %w", path, i, p, err)
			}
			buffers[i] = data
		default:
			return nil, fmt.Errorf("%s: buffer %d has no uri and no embedded BIN chunk", path, i)
		}
		if len(buffers[i]) < b.ByteLength {
			return nil, fmt.Errorf("%s: buffer %d is %d bytes, expected at least %d", path, i, len(buffers[i]), b.ByteLength)
		}
	}

	return &LoadedGLTF{Doc: doc, Buffers: buffers}, nil
}

// parseGLBContainer splits a .glb file into its JSON chunk and (optional)
// binary chunk, per the real GLB container spec: 12-byte header (magic,
// version, total length), then a sequence of (chunkLength, chunkType,
// data) chunks -- JSON always first, at most one BIN chunk after it.
func parseGLBContainer(raw []byte) (jsonChunk, binChunk []byte, err error) {
	version := binary.LittleEndian.Uint32(raw[4:8])
	if version != 2 {
		return nil, nil, fmt.Errorf("unsupported GLB version %d (only 2 is supported)", version)
	}
	totalLen := binary.LittleEndian.Uint32(raw[8:12])
	if int(totalLen) > len(raw) {
		return nil, nil, fmt.Errorf("GLB declares length %d, file is only %d bytes", totalLen, len(raw))
	}

	off := 12
	for off+8 <= len(raw) {
		chunkLen := int(binary.LittleEndian.Uint32(raw[off : off+4]))
		chunkType := binary.LittleEndian.Uint32(raw[off+4 : off+8])
		dataStart := off + 8
		dataEnd := dataStart + chunkLen
		if dataEnd > len(raw) {
			return nil, nil, fmt.Errorf("GLB chunk at offset %d overruns file (len %d, have %d)", off, chunkLen, len(raw)-dataStart)
		}
		data := raw[dataStart:dataEnd]
		switch chunkType {
		case glbChunkJSON:
			jsonChunk = data
		case glbChunkBinary:
			binChunk = data
		}
		off = dataEnd
	}
	if jsonChunk == nil {
		return nil, nil, fmt.Errorf("GLB file has no JSON chunk")
	}
	return jsonChunk, binChunk, nil
}

// decodeDataURI decodes a "data:<mime>;base64,<payload>" URI -- the only
// form Blender's exporter (or any real glTF tool) ever emits for an
// embedded buffer.
func decodeDataURI(uri string) ([]byte, error) {
	comma := strings.IndexByte(uri, ',')
	if comma < 0 || !strings.Contains(uri[:comma], "base64") {
		return nil, fmt.Errorf("unsupported data URI (only base64 is handled): %.40s...", uri)
	}
	return base64.StdEncoding.DecodeString(uri[comma+1:])
}

// accessorFloats decodes accessor `idx` into Count rows of componentsPerElement
// float64 values each, upconverting from whatever componentType the
// accessor actually declares. Sparse accessors are not supported (v0 --
// none of this pipeline's real Blender exports use them).
func (g *LoadedGLTF) accessorFloats(idx int) ([][]float64, error) {
	if idx < 0 || idx >= len(g.Doc.Accessors) {
		return nil, fmt.Errorf("accessor index %d out of range", idx)
	}
	acc := g.Doc.Accessors[idx]
	numComp, ok := gltfTypeComponents[acc.Type]
	if !ok {
		return nil, fmt.Errorf("accessor %d: unsupported type %q", idx, acc.Type)
	}
	if acc.BufferView == nil {
		return nil, fmt.Errorf("accessor %d: sparse/bufferView-less accessors are not supported", idx)
	}
	bv := g.Doc.BufferViews[*acc.BufferView]
	buf := g.Buffers[bv.Buffer]

	compSize, err := gltfComponentSize(acc.ComponentType)
	if err != nil {
		return nil, fmt.Errorf("accessor %d: %w", idx, err)
	}
	elemSize := compSize * numComp
	stride := bv.ByteStride
	if stride == 0 {
		stride = elemSize
	}

	base := bv.ByteOffset + acc.ByteOffset
	rows := make([][]float64, acc.Count)
	for i := 0; i < acc.Count; i++ {
		off := base + i*stride
		row := make([]float64, numComp)
		for c := 0; c < numComp; c++ {
			row[c], err = gltfReadComponent(buf, off+c*compSize, acc.ComponentType)
			if err != nil {
				return nil, fmt.Errorf("accessor %d, element %d, component %d: %w", idx, i, c, err)
			}
		}
		rows[i] = row
	}
	return rows, nil
}

func gltfComponentSize(componentType int) (int, error) {
	switch componentType {
	case gltfByte, gltfUnsignedByte:
		return 1, nil
	case gltfShort, gltfUnsignedShort:
		return 2, nil
	case gltfUnsignedInt, gltfFloat:
		return 4, nil
	default:
		return 0, fmt.Errorf("unsupported componentType %d", componentType)
	}
}

func gltfReadComponent(buf []byte, off, componentType int) (float64, error) {
	switch componentType {
	case gltfFloat:
		if off+4 > len(buf) {
			return 0, fmt.Errorf("read past end of buffer")
		}
		return float64(math.Float32frombits(binary.LittleEndian.Uint32(buf[off : off+4]))), nil
	case gltfUnsignedInt:
		if off+4 > len(buf) {
			return 0, fmt.Errorf("read past end of buffer")
		}
		return float64(binary.LittleEndian.Uint32(buf[off : off+4])), nil
	case gltfUnsignedShort:
		if off+2 > len(buf) {
			return 0, fmt.Errorf("read past end of buffer")
		}
		return float64(binary.LittleEndian.Uint16(buf[off : off+2])), nil
	case gltfUnsignedByte:
		if off+1 > len(buf) {
			return 0, fmt.Errorf("read past end of buffer")
		}
		return float64(buf[off]), nil
	default:
		return 0, fmt.Errorf("unsupported componentType %d", componentType)
	}
}
