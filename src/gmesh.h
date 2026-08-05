// gmesh.h — S144-07: the GOLDEN BAND skinned-mesh asset loader.
//
// Same philosophy as gband.h/gskel.h: no engine dependency, pure data.
// Skinning (blending bone_indices/bone_weights against a posed GSkel into
// world-space vertices) is the consuming engine's job -- see
// format/GMESH_FORMAT.md's own note on why that math lives in the
// consumer's bridge code, not here. See format/GMESH_FORMAT.md for the
// full binary layout.
#ifndef GOLDENBAND_GMESH_H
#define GOLDENBAND_GMESH_H

#include <stdint.h>

typedef struct {
    float position[3]; // bind pose, rest-pose world space (matches .gskel's inverse_bind space)
    float normal[3];   // bind pose
    float uv[2];        // reserved, unused until a texturing pass exists
    uint8_t bone_indices[4]; // indices into the paired GSkel's joints[]
    float bone_weights[4];   // normalized, sums to ~1.0 (exporter's responsibility)
} GMeshVertex;

typedef struct {
    uint32_t version;
    uint32_t vertex_count;
    uint32_t index_count;   // always a multiple of 3
    GMeshVertex *vertices;  // owned, vertex_count entries
    uint32_t *indices;      // owned, index_count entries, into vertices[]
} GMesh;

// gmesh_init loads and structurally validates a .gmesh file. Returns 1 on
// success, 0 on any failure. On success, caller must eventually call
// gmesh_free.
int gmesh_init(const char *path, GMesh *mesh);

void gmesh_free(GMesh *mesh);

#endif // GOLDENBAND_GMESH_H
