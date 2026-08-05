// gmesh.c — see gmesh.h. Manual byte-offset parsing, same rationale as
// gskel.c (no reliance on compiler struct layout matching the file layout).
#include "gmesh.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GMESH_HEADER_SIZE 16
#define GMESH_VERTEX_RECORD_SIZE 52

static uint32_t read_u32le(const unsigned char *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float read_f32le(const unsigned char *p) {
    uint32_t bits = read_u32le(p);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

int gmesh_init(const char *path, GMesh *mesh) {
    memset(mesh, 0, sizeof(*mesh));

    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    unsigned char header[GMESH_HEADER_SIZE];
    if (fread(header, 1, GMESH_HEADER_SIZE, f) != GMESH_HEADER_SIZE) { fclose(f); return 0; }
    if (memcmp(header, "GMSH", 4) != 0) { fclose(f); return 0; }

    mesh->version = read_u32le(header + 4);
    mesh->vertex_count = read_u32le(header + 8);
    mesh->index_count = read_u32le(header + 12);
    if (mesh->vertex_count == 0 || mesh->index_count == 0 || mesh->index_count % 3 != 0) {
        fclose(f);
        return 0;
    }

    mesh->vertices = (GMeshVertex *)malloc((size_t)mesh->vertex_count * sizeof(GMeshVertex));
    mesh->indices = (uint32_t *)malloc((size_t)mesh->index_count * sizeof(uint32_t));
    if (!mesh->vertices || !mesh->indices) {
        free(mesh->vertices); free(mesh->indices);
        mesh->vertices = NULL; mesh->indices = NULL;
        fclose(f);
        return 0;
    }

    unsigned char rec[GMESH_VERTEX_RECORD_SIZE];
    for (uint32_t v = 0; v < mesh->vertex_count; v++) {
        if (fread(rec, 1, GMESH_VERTEX_RECORD_SIZE, f) != GMESH_VERTEX_RECORD_SIZE) {
            free(mesh->vertices); free(mesh->indices);
            mesh->vertices = NULL; mesh->indices = NULL;
            fclose(f);
            return 0;
        }
        GMeshVertex *vert = &mesh->vertices[v];
        for (int k = 0; k < 3; k++) vert->position[k] = read_f32le(rec + 0 + k * 4);
        for (int k = 0; k < 3; k++) vert->normal[k] = read_f32le(rec + 12 + k * 4);
        for (int k = 0; k < 2; k++) vert->uv[k] = read_f32le(rec + 24 + k * 4);
        for (int k = 0; k < 4; k++) vert->bone_indices[k] = rec[32 + k];
        for (int k = 0; k < 4; k++) vert->bone_weights[k] = read_f32le(rec + 36 + k * 4);
    }

    unsigned char idx4[4];
    for (uint32_t i = 0; i < mesh->index_count; i++) {
        if (fread(idx4, 1, 4, f) != 4) {
            free(mesh->vertices); free(mesh->indices);
            mesh->vertices = NULL; mesh->indices = NULL;
            fclose(f);
            return 0;
        }
        mesh->indices[i] = read_u32le(idx4);
    }

    fclose(f);
    return 1;
}

void gmesh_free(GMesh *mesh) {
    free(mesh->vertices);
    free(mesh->indices);
    mesh->vertices = NULL;
    mesh->indices = NULL;
}
