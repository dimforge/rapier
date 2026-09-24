/* Position/index subset of OBJ used by the Rust mesh examples. No runtime dependency. */
#ifndef EXAMPLE_OBJ_H
#define EXAMPLE_OBJ_H
#include "testbed.h"
#include "rapier_math.h"
#include <ctype.h>
#include <errno.h>
#include <limits.h>

typedef struct ObjMesh {
    R3Vector *vertices;
    uint32_t *indices;
    size_t vertexCount, indexCount;
} ObjMesh;

static void freeObj(ObjMesh *mesh) {
    free(mesh->vertices);
    free(mesh->indices);
    *mesh = (ObjMesh){0};
}

/* Preserve polygon index order, as obj::raw::parse_obj does in the Rust examples. */
static int loadObj(const char *root, const char *name, ObjMesh *mesh) {
    char path[4096];
    if (snprintf(path, sizeof(path), "%s/3d/%s", root, name) >= (int)sizeof(path)) {
        return 0;
    }
    FILE *file = fopen(path, "rb");
    if (!file) {
        fprintf(stderr, "Cannot open %s: %s\n", path, strerror(errno));
        return 0;
    }
    *mesh = (ObjMesh){0};
    char *line = NULL;
    size_t capacity = 0, vertexCapacity = 0, indexCapacity = 0;
    int valid = 1;
    while (valid) {
        size_t length = 0;
        int ch;
        while ((ch = fgetc(file)) != EOF && ch != '\n') {
            if (length + 1 >= capacity) {
                capacity = capacity ? capacity * 2 : 256;
                char *next = realloc(line, capacity);
                if (!next) {
                    abort();
                }
                line = next;
            }
            line[length++] = (char)ch;
        }
        if (ch == EOF && !length) {
            break;
        }
        if (!line) {
            capacity = 256;
            line = malloc(capacity);
            if (!line) {
                abort();
            }
        }
        line[length] = '\0';
        char *p = line;
        while (isspace((unsigned char)*p)) {
            ++p;
        }
        if (p[0] == 'v' && isspace((unsigned char)p[1])) {
            double x, y, z;
            if (sscanf(p + 1, "%lf %lf %lf", &x, &y, &z) != 3) {
                valid = 0;
                break;
            }
            if (mesh->vertexCount == vertexCapacity) {
                vertexCapacity = vertexCapacity ? vertexCapacity * 2 : 256;
                R3Vector *next = realloc(mesh->vertices, vertexCapacity * sizeof(*next));
                if (!next) {
                    abort();
                }
                mesh->vertices = next;
            }
            mesh->vertices[mesh->vertexCount++] = r3Vector(x, y, z);
        } else if (p[0] == 'f' && isspace((unsigned char)p[1])) {
            ++p;
            while (*p) {
                while (isspace((unsigned char)*p)) {
                    ++p;
                }
                if (!*p || *p == '#') {
                    break;
                }
                char *end;
                long index = strtol(p, &end, 10);
                if (end == p || index == 0) {
                    valid = 0;
                    break;
                }
                long vertex = index > 0 ? index - 1 : (long)mesh->vertexCount + index;
                if (vertex < 0 || (size_t)vertex >= mesh->vertexCount ||
                    (unsigned long)vertex > UINT32_MAX) {
                    valid = 0;
                    break;
                }
                if (mesh->indexCount == indexCapacity) {
                    indexCapacity = indexCapacity ? indexCapacity * 2 : 768;
                    uint32_t *next = realloc(mesh->indices, indexCapacity * sizeof(*next));
                    if (!next) {
                        abort();
                    }
                    mesh->indices = next;
                }
                mesh->indices[mesh->indexCount++] = (uint32_t)vertex;
                p = end;
                while (*p && !isspace((unsigned char)*p)) {
                    ++p;
                }
            }
        }
    }
    valid = valid && !ferror(file) && mesh->vertexCount && mesh->indexCount &&
            mesh->indexCount % 3 == 0;
    free(line);
    fclose(file);
    if (!valid) {
        fprintf(stderr, "Invalid triangle OBJ: %s\n", path);
        freeObj(mesh);
    }
    return valid;
}
#endif
