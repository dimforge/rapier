#include "testbed_internal.h"
#include "graphics.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdlib.h>
#include <limits.h>

/* Cache by collider generation and immutable shape identity. Geometry is deduplicated
 * by content so separately constructed equal shapes still share instanced draws. */
typedef struct Geometry {
    Mesh mesh;
    RAPIER_TYPE(Vector) *triangles, *lines;
    size_t nt, nl;
    uint64_t hash;
    bool allocated, live;
} Geometry;

typedef struct Entry {
    uint32_t generation;
    uintptr_t identity;
    RAPIER_TYPE(SharedShape) *shape;
    size_t geometry;
    bool valid, soft, seen;
} Entry;

typedef struct Batch {
    size_t geometry;
    Color color;
    Matrix *transforms;
    size_t count, capacity;
} Batch;

typedef struct TransparentDraw {
    size_t batch, instance;
    float depth;
    bool triangle;
    Vector3 vertices[3];
    Color color;
} TransparentDraw;

typedef struct VisualMesh {
    Mesh mesh;
    Material material;
    Texture2D texture;
} VisualMesh;

struct TbGraphics {
    Geometry *geometries;
    size_t geometryCount, geometryCapacity;
    Entry *entries;
    size_t entryCapacity;
    Batch *batches;
    size_t batchCount, batchCapacity;
    TransparentDraw *transparent;
    size_t transparentCount, transparentCapacity;
    RAPIER_TYPE(ColliderHandle) *handles;
    RAPIER_TYPE(SoftMeshInfo) *softMeshes;
    size_t handlesCapacity, softMeshCapacity;
    RAPIER_TYPE(SoftBodyHandle) *softHandles;
    size_t softHandlesCapacity;
    RAPIER_TYPE(Vector) *vertices;
    size_t vertexCapacity;
    uint32_t *indices;
    size_t indexCapacity;
    RAPIER_TYPE(DebugLine) *debug;
    size_t debugCapacity;
    VisualMesh *visuals;
    size_t visualCount;
    Shader visualShader;
    Shader shader;
    Material material;
};

static void *grow(void *p, size_t *cap, size_t n, size_t elem) {
    if (n <= *cap) {
        return p;
    }
    size_t old = *cap;
    size_t next = old ? old * 2 : 16;
    if (next < n) {
        next = n;
    }
    if (next > SIZE_MAX / elem) {
        abort();
    }
    void *q = realloc(p, next * elem);
    if (!q) {
        abort();
    }
    memset((char *)q + old * elem, 0, (next - old) * elem);
    *cap = next;
    return q;
}

static Vector3 vec(RAPIER_TYPE(Vector) p) {
#if defined(RAPIER_DIM3)
    return (Vector3){(float)p.x, (float)p.y, (float)p.z};
#else
    return (Vector3){(float)p.x, (float)p.y, 0};
#endif
}

static Matrix transform(RAPIER_TYPE(Pose) p) {
#if defined(RAPIER_DIM3)
    Matrix m = QuaternionToMatrix((Quaternion){(float)p.rotation.x, (float)p.rotation.y,
                                               (float)p.rotation.z, (float)p.rotation.w});
#else
    Matrix m = MatrixRotateZ((float)p.rotation.angle);
#endif
    Vector3 v = vec(p.translation);
    m.m12 = v.x;
    m.m13 = v.y;
    m.m14 = v.z;
    return m;
}

static const Color palette[] = {{82, 130, 150, 255},  {222, 139, 89, 255}, {121, 165, 108, 255},
                                {159, 133, 184, 255}, {217, 180, 75, 255}, {94, 162, 164, 255},
                                {193, 115, 136, 255}, {132, 149, 178, 255}};

static Color color(Testbed *t, RAPIER_TYPE(ColliderHandle) h) {
    RAPIER_TYPE(RigidBodyHandle) p;
    RAPIER_TYPE(Bool) sensor;
    p = RAPIER_FN(Collider_Parent)(h);
    TB(t, RAPIER_FN(LastStatus)());
    sensor = RAPIER_FN(Collider_IsSensor)(h);
    TB(t, RAPIER_FN(LastStatus)());
    Color result = palette[(p.index == UINT32_MAX ? h.index : p.index) % TB_COUNT(palette)];
    if (p.index == UINT32_MAX) {
        result = (Color){170, 174, 175, 255};
    } else {
        RAPIER_TYPE(Bool) fixed;
        TB(t, RAPIER_FN(RigidBody_ValidateHandle)(p));
        fixed = RAPIER_FN(RigidBody_IsFixed)(p);
        TB(t, RAPIER_FN(LastStatus)());
        if (fixed) {
            result = (Color){170, 174, 175, 255};
        }
    }
    const float *rgba = tbFindColor(t, p, h);
    if (rgba) {
        result = (Color){(unsigned char)(rgba[0] * 255), (unsigned char)(rgba[1] * 255),
                         (unsigned char)(rgba[2] * 255), (unsigned char)(rgba[3] * 255)};
    }
    if (sensor) {
        result.a = (unsigned char)(result.a * 0.4f);
    }
    return result;
}

TbGraphics *tbGraphicsNew(void) {
    TbGraphics *g = calloc(1, sizeof(*g));
    if (!g) {
        abort();
    }
    const char *vs =
        "#version 330\nin vec3 vertexPosition;in vec3 vertexNormal;in mat4 "
        "instanceTransform;uniform mat4 mvp;out float light;void main(){vec3 "
        "n=normalize(mat3(instanceTransform)*vertexNormal);light=0.5+0.5*abs(dot(n,normalize(vec3("
        "0.5,0.8,0.6))));gl_Position=mvp*instanceTransform*vec4(vertexPosition,1.0);}";
    const char *fs = "#version 330\nin float light;uniform vec4 colDiffuse;out vec4 "
                     "finalColor;void main(){finalColor=vec4(colDiffuse.rgb*light,colDiffuse.a);}";
    g->shader = LoadShaderFromMemory(vs, fs);
    if (!IsShaderValid(g->shader)) {
        fputs("instancing shader failed\n", stderr);
        abort();
    }
    g->shader.locs[SHADER_LOC_VERTEX_INSTANCETRANSFORM] =
        GetShaderLocationAttrib(g->shader, "instanceTransform");
    const char *visualVs =
        "#version 330\nin vec3 vertexPosition;in vec3 vertexNormal;in vec2 vertexTexCoord;"
        "in mat4 instanceTransform;uniform mat4 mvp;out vec3 position;out vec3 normal;out vec2 uv;"
        "void main(){position=vec3(instanceTransform*vec4(vertexPosition,1));"
        "normal=mat3(instanceTransform)*vertexNormal;uv=vertexTexCoord;"
        "gl_Position=mvp*vec4(position,1);}";
    const char *visualFs =
        "#version 330\nin vec3 position;in vec3 normal;in vec2 uv;out vec4 finalColor;"
        "uniform sampler2D texture0;uniform vec4 colDiffuse;uniform vec3 eye;"
        "uniform float metallic;uniform float roughness;uniform float reflectance;uniform vec3 "
        "emissive;"
        "void main(){vec4 base=texture(texture0,uv)*colDiffuse;vec3 n=normalize(normal);"
        "if(!gl_FrontFacing)n=-n;vec3 l=normalize(vec3(0.5,0.8,0.6));vec3 "
        "v=normalize(eye-position);"
        "vec3 h=normalize(l+v);float nl=max(dot(n,l),0.0),nv=max(dot(n,v),0.001);"
        "float nh=max(dot(n,h),0.0),vh=max(dot(v,h),0.0);float a=max(roughness*roughness,0.002);"
        "float a2=a*a,d=nh*nh*(a2-1.0)+1.0;float D=a2/(3.141593*d*d);"
        "float k=(roughness+1.0)*(roughness+1.0)/8.0;"
        "float G=nl/(nl*(1.0-k)+k)*nv/(nv*(1.0-k)+k);"
        "vec3 F0=mix(vec3(0.16*reflectance*reflectance),base.rgb,metallic);"
        "vec3 F=F0+(1.0-F0)*pow(1.0-vh,5.0);"
        "vec3 diffuse=(1.0-F)*(1.0-metallic)*base.rgb/3.141593;"
        "vec3 specular=D*G*F/max(4.0*nl*nv,0.001);"
        "vec3 color=base.rgb*0.22+(diffuse+specular)*nl*2.5+emissive;"
        "finalColor=vec4(color,base.a);}";
    g->visualShader = LoadShaderFromMemory(visualVs, visualFs);
    if (!IsShaderValid(g->visualShader)) {
        abort();
    }
    g->visualShader.locs[SHADER_LOC_VERTEX_INSTANCETRANSFORM] =
        GetShaderLocationAttrib(g->visualShader, "instanceTransform");
    g->material = LoadMaterialDefault();
    g->material.shader = g->shader;
    return g;
}

void tbGraphicsFree(TbGraphics *g) {
    if (!g) {
        return;
    }
    for (size_t i = 0; i < g->visualCount; ++i) {
        UnloadMesh(g->visuals[i].mesh);
        if (g->visuals[i].texture.id) {
            UnloadTexture(g->visuals[i].texture);
        }
        MemFree(g->visuals[i].material.maps);
    }
    free(g->visuals);
    UnloadShader(g->visualShader);
    for (size_t i = 0; i < g->geometryCount; i++) {
        Geometry *m = &g->geometries[i];
        if (m->nt) {
            UnloadMesh(m->mesh);
        }
        free(m->triangles);
        free(m->lines);
    }
    for (size_t i = 0; i < g->entryCapacity; i++) {
        (void)RAPIER_FN(FreeSharedShape)(g->entries[i].shape);
    }
    for (size_t i = 0; i < g->batchCount; i++) {
        free(g->batches[i].transforms);
    }
    free(g->geometries);
    free(g->entries);
    free(g->batches);
    free(g->transparent);
    free(g->handles);
    free(g->softMeshes);
    free(g->softHandles);
    free(g->vertices);
    free(g->indices);
    free(g->debug); /* shader is owned separately from Material */
    g->material.shader = (Shader){0};
    UnloadMaterial(g->material);
    UnloadShader(g->shader);
    free(g);
}

static uint64_t hashBytes(uint64_t h, const void *v, size_t n) {
    const unsigned char *p = v;
    while (n--) {
        h = (h ^ *p++) * UINT64_C(1099511628211);
    }
    return h;
}

static size_t geometry(TbGraphics *g, Testbed *t, RAPIER_TYPE(SharedShape) *shape) {
    RAPIER_TYPE(ShapeMesh) *source = RAPIER_FN(SharedShape_Tessellate)(shape, 16);
    TB(t, RAPIER_FN(LastStatus)());
    size_t nt = 0, nl = 0;
    nt = RAPIER_FN(ShapeMesh_Triangles)(source, NULL, 0);
    TB(t, RAPIER_FN(LastStatus)());
    nl = RAPIER_FN(ShapeMesh_Lines)(source, NULL, 0);
    TB(t, RAPIER_FN(LastStatus)());
    RAPIER_TYPE(Vector) *tri = calloc(nt ? nt : 1, sizeof(*tri)),
                        *lines = calloc(nl ? nl : 1, sizeof(*lines));
    if (!tri || !lines) {
        abort();
    }
    nt = RAPIER_FN(ShapeMesh_Triangles)(source, tri, nt);
    TB(t, RAPIER_FN(LastStatus)());
    nl = RAPIER_FN(ShapeMesh_Lines)(source, lines, nl);
    TB(t, RAPIER_FN(LastStatus)());
    TB(t, RAPIER_FN(FreeShapeMesh)(source));
    uint64_t hash = hashBytes(hashBytes(UINT64_C(14695981039346656037), tri, nt * sizeof(*tri)),
                              lines, nl * sizeof(*lines));
    for (size_t i = 0; i < g->geometryCount; i++) {
        Geometry *m = &g->geometries[i];
        if (m->allocated && m->hash == hash && m->nt == nt && m->nl == nl &&
            !memcmp(m->triangles, tri, nt * sizeof(*tri)) &&
            !memcmp(m->lines, lines, nl * sizeof(*lines))) {
            free(tri);
            free(lines);
            return i;
        }
    }
    g->geometries =
        grow(g->geometries, &g->geometryCapacity, g->geometryCount + 1, sizeof(*g->geometries));
    size_t id = 0;
    while (id < g->geometryCount && g->geometries[id].allocated) {
        id++;
    }
    if (id == g->geometryCount) {
        g->geometryCount++;
    }
    Geometry *m = &g->geometries[id];
    m->allocated = true;
    m->triangles = tri;
    m->lines = lines;
    m->nt = nt;
    m->nl = nl;
    m->hash = hash;
    if (nt) {
        if (nt > INT_MAX / 3) {
            abort();
        }
        m->mesh.vertexCount = (int)nt;
        m->mesh.triangleCount = (int)(nt / 3);
        m->mesh.vertices = MemAlloc((unsigned int)(nt * 3 * sizeof(float)));
        m->mesh.normals = MemAlloc((unsigned int)(nt * 3 * sizeof(float)));
        if (!m->mesh.vertices || !m->mesh.normals) {
            abort();
        }
        for (size_t i = 0; i < nt; i += 3) {
            Vector3 a = vec(tri[i]), b = vec(tri[i + 1]), c = vec(tri[i + 2]);
            Vector3 n =
                Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
            for (size_t j = 0; j < 3; j++) {
                Vector3 v = vec(tri[i + j]);
                memcpy(&m->mesh.vertices[(i + j) * 3], &v, sizeof(v));
                memcpy(&m->mesh.normals[(i + j) * 3], &n, sizeof(n));
            }
        }
        UploadMesh(&m->mesh, false);
    }
    return id;
}

static void instance(TbGraphics *g, size_t geometryId, Color c, Matrix m) {
    Batch *b = NULL;
    for (size_t i = 0; i < g->batchCount; i++) {
        if (g->batches[i].geometry == geometryId && !memcmp(&g->batches[i].color, &c, sizeof(c))) {
            b = &g->batches[i];
            break;
        }
    }
    if (!b) {
        g->batches = grow(g->batches, &g->batchCapacity, g->batchCount + 1, sizeof(*g->batches));
        b = &g->batches[g->batchCount++];
        memset(b, 0, sizeof(*b));
        b->geometry = geometryId;
        b->color = c;
    }
    b->transforms = grow(b->transforms, &b->capacity, b->count + 1, sizeof(*b->transforms));
    b->transforms[b->count++] = m;
}

static Entry *entry(TbGraphics *g, RAPIER_TYPE(ColliderHandle) h) {
    /* A render-only mesh has no collider and must never index this cache. */
    if (h.index == UINT32_MAX) {
        abort();
    }
    g->entries = grow(g->entries, &g->entryCapacity, (size_t)h.index + 1, sizeof(*g->entries));
    return &g->entries[h.index];
}

/* Deforming triangles share the same sorted transparency pass as rigid sensors. */
static void softTriangle(TbGraphics *g, Vector3 a, Vector3 b, Vector3 c, Color color,
                         Camera3D camera) {
    if (color.a == 255) {
        DrawTriangle3D(a, b, c, color);
        return;
    }
    Vector3 center = Vector3Scale(Vector3Add(Vector3Add(a, b), c), 1.0f / 3);
    Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    g->transparent = grow(g->transparent, &g->transparentCapacity, g->transparentCount + 1,
                          sizeof(*g->transparent));
    g->transparent[g->transparentCount++] = (TransparentDraw){
        .depth = Vector3DotProduct(Vector3Subtract(center, camera.position), forward),
        .triangle = true,
        .vertices = {a, b, c},
        .color = color};
}

/* Screen-facing ribbons keep deformable polylines readable on every GL backend,
 * including those that only support one-pixel native lines. */
static void drawSoftSegment(TbGraphics *g, Vector3 a, Vector3 b, Color color, Camera3D camera) {
    Vector3 middle = Vector3Scale(Vector3Add(a, b), .5f);
    Vector3 view = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    Vector3 side = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), view));
    float height = fmaxf(1, (float)GetScreenHeight());
    float worldPerPixel = camera.fovy / height;
    if (camera.projection == CAMERA_PERSPECTIVE) {
        float depth = Vector3DotProduct(Vector3Subtract(middle, camera.position), view);
        worldPerPixel = 2 * fmaxf(.001f, depth) * tanf(camera.fovy * DEG2RAD * .5f) / height;
    }
    side = Vector3Scale(side, worldPerPixel * 1.5f); /* Three pixels wide. */
    Vector3 a0 = Vector3Subtract(a, side), a1 = Vector3Add(a, side);
    Vector3 b0 = Vector3Subtract(b, side), b1 = Vector3Add(b, side);
    softTriangle(g, a0, b0, b1, color, camera);
    softTriangle(g, a0, b1, a1, color, camera);
}

static void drawSoft(TbGraphics *g, Testbed *t, bool surfaces, Camera3D camera) {
    size_t n = RAPIER_FN(SoftBodyHandles)(t->world, NULL, 0);
    TB(t, RAPIER_FN(LastStatus)());
    g->softHandles = grow(g->softHandles, &g->softHandlesCapacity, n, sizeof(*g->softHandles));
    n = RAPIER_FN(SoftBodyHandles)(t->world, g->softHandles, g->softHandlesCapacity);
    TB(t, RAPIER_FN(LastStatus)());
    for (size_t i = 0; i < n; i++) {
        TB(t, RAPIER_FN(SoftBody_ValidateHandle)(g->softHandles[i]));
        size_t nm = RAPIER_FN(SoftBody_Meshes)(g->softHandles[i], NULL, 0);
        TB(t, RAPIER_FN(LastStatus)());
        g->softMeshes = grow(g->softMeshes, &g->softMeshCapacity, nm, sizeof(*g->softMeshes));
        nm = RAPIER_FN(SoftBody_Meshes)(g->softHandles[i], g->softMeshes,
                                       g->softMeshCapacity);
        TB(t, RAPIER_FN(LastStatus)());
        bool drawnSkin = false;
        for (size_t j = 0; j < nm; ++j) {
            drawnSkin |= g->softMeshes[j].is_skinned && !g->softMeshes[j].collision_enabled;
        }
        for (size_t j = 0; j < nm; j++) {
            const RAPIER_TYPE(SoftMeshInfo) *mesh = &g->softMeshes[j];
            RAPIER_TYPE(ColliderHandle) h = mesh->collider;
            if (h.index != UINT32_MAX) {
                entry(g, h)->soft = true;
                entry(g, h)->seen = true;
            }
            /* Match the Rust viewer: drawn skins replace collision boundaries;
             * non-colliding computational boundaries are not render objects. */
            if (!surfaces || (!mesh->collision_enabled && !mesh->is_skinned) ||
                (drawnSkin && mesh->collision_enabled)) {
                continue;
            }
            size_t nv = 0, ni = 0, arity = mesh->arity;
            nv =
                RAPIER_FN(SoftBody_MeshVerticesById)(g->softHandles[i], mesh->id, NULL, 0);
            TB(t, RAPIER_FN(LastStatus)());
            ni = RAPIER_FN(SoftBody_MeshIndicesById)(g->softHandles[i], mesh->id, NULL, 0);
            TB(t, RAPIER_FN(LastStatus)());
            g->vertices = grow(g->vertices, &g->vertexCapacity, nv, sizeof(*g->vertices));
            g->indices = grow(g->indices, &g->indexCapacity, ni, sizeof(*g->indices));
            nv = RAPIER_FN(SoftBody_MeshVerticesById)(g->softHandles[i], mesh->id,
                                                     g->vertices, g->vertexCapacity);
            TB(t, RAPIER_FN(LastStatus)());
            ni = RAPIER_FN(SoftBody_MeshIndicesById)(g->softHandles[i], mesh->id,
                                                    g->indices, g->indexCapacity);
            TB(t, RAPIER_FN(LastStatus)());
            Color c = palette[g->softHandles[i].index % TB_COUNT(palette)];
            RAPIER_TYPE(RigidBodyHandle)
            root = RAPIER_FN(SoftBody_RootBody)(g->softHandles[i]);
            TB(t, RAPIER_FN(LastStatus)());
            const float *tint = tbFindColor(t, root, h);
            if (tint) {
                c = (Color){(unsigned char)(tint[0] * 255), (unsigned char)(tint[1] * 255),
                            (unsigned char)(tint[2] * 255), (unsigned char)(tint[3] * 255)};
            }
            if (h.index != UINT32_MAX) {
                RAPIER_TYPE(Bool) sensor;
                TB(t, RAPIER_FN(Collider_ValidateHandle)(h));
                sensor = RAPIER_FN(Collider_IsSensor)(h);
                TB(t, RAPIER_FN(LastStatus)());
                if (sensor) {
                    c.a = (unsigned char)(c.a * .4f);
                }
            }
            if (arity == 3) {
                for (size_t k = 0; k + 2 < ni; k += 3) {
                    Vector3 a = vec(g->vertices[g->indices[k]]),
                            b = vec(g->vertices[g->indices[k + 1]]),
                            v = vec(g->vertices[g->indices[k + 2]]);
                    Vector3 norm = Vector3Normalize(
                        Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(v, a)));
                    float light =
                        0.5f + 0.5f * fabsf(Vector3DotProduct(
                                          norm, Vector3Normalize((Vector3){0.5f, 0.8f, 0.6f})));
                    softTriangle(g, a, b, v,
                                 (Color){(unsigned char)(c.r * light), (unsigned char)(c.g * light),
                                         (unsigned char)(c.b * light), c.a},
                                 camera);
                }
            } else {
                for (size_t k = 0; k + 1 < ni; k += 2) {
                    drawSoftSegment(g, vec(g->vertices[g->indices[k]]),
                                    vec(g->vertices[g->indices[k + 1]]), c, camera);
                }
            }
        }
    }
}

static void drawVisualMeshes(TbGraphics *g, Testbed *t, Camera3D camera) {
    if (!g->visuals && t->renderMeshCount) {
        g->visuals = calloc(t->renderMeshCount, sizeof(*g->visuals));
        if (!g->visuals) {
            abort();
        }
        g->visualCount = t->renderMeshCount;
        for (size_t i = 0; i < g->visualCount; ++i) {
            const TbRenderMesh *source = &t->renderMeshes[i];
            VisualMesh *visual = &g->visuals[i];
            Mesh *mesh = &visual->mesh;
            if (source->indexCount > INT_MAX ||
                source->indexCount > UINT_MAX / (3 * sizeof(float))) {
                abort();
            }
            mesh->vertexCount = (int)source->indexCount;
            mesh->triangleCount = mesh->vertexCount / 3;
            mesh->vertices = MemAlloc((unsigned)(source->indexCount * 3 * sizeof(float)));
            mesh->normals = MemAlloc((unsigned)(source->indexCount * 3 * sizeof(float)));
            mesh->texcoords = MemAlloc((unsigned)(source->indexCount * 2 * sizeof(float)));
            if (!mesh->vertices || !mesh->normals || !mesh->texcoords) {
                abort();
            }
            for (size_t j = 0; j < source->indexCount; j += 3) {
                Vector3 a = vec(source->vertices[source->indices[j]]);
                Vector3 b = vec(source->vertices[source->indices[j + 1]]);
                Vector3 c = vec(source->vertices[source->indices[j + 2]]);
                Vector3 normal = Vector3Normalize(
                    Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
                for (size_t k = j; k < j + 3; ++k) {
                    uint32_t id = source->indices[k];
                    Vector3 vertex = vec(source->vertices[id]);
                    memcpy(mesh->vertices + k * 3, &vertex, sizeof(vertex));
                    memcpy(mesh->normals + k * 3,
                           source->normals ? source->normals + id * 3 : (float *)&normal,
                           3 * sizeof(float));
                    mesh->texcoords[k * 2] = source->uvs ? source->uvs[id * 2] : 0;
                    mesh->texcoords[k * 2 + 1] = source->uvs ? source->uvs[id * 2 + 1] : 0;
                }
            }
            UploadMesh(mesh, false);
            visual->material = LoadMaterialDefault();
            visual->material.shader = g->visualShader;
            const float *color = source->rgba;
            visual->material.maps[MATERIAL_MAP_DIFFUSE].color =
                (Color){(unsigned char)(color[0] * 255), (unsigned char)(color[1] * 255),
                        (unsigned char)(color[2] * 255), (unsigned char)(color[3] * 255)};
            if (source->texture && source->texture[0]) {
                visual->texture = LoadTexture(source->texture);
                if (visual->texture.id) {
                    visual->material.maps[MATERIAL_MAP_DIFFUSE].texture = visual->texture;
                }
            }
        }
    }
    SetShaderValue(g->visualShader, GetShaderLocation(g->visualShader, "eye"), &camera.position,
                   SHADER_UNIFORM_VEC3);
    for (size_t i = 0; i < g->visualCount; ++i) {
        const TbRenderMesh *source = &t->renderMeshes[i];

        RAPIER_TYPE(Pose) pose;
        TB(t, RAPIER_FN(RigidBody_ValidateHandle)(source->body));
        pose = RAPIER_FN(RigidBody_Position)(source->body);
        TB(t, RAPIER_FN(LastStatus)());
        Matrix matrix = MatrixMultiply(transform(source->localPose), transform(pose));
        SetShaderValue(g->visualShader, GetShaderLocation(g->visualShader, "metallic"),
                       &source->metallic, SHADER_UNIFORM_FLOAT);
        SetShaderValue(g->visualShader, GetShaderLocation(g->visualShader, "roughness"),
                       &source->roughness, SHADER_UNIFORM_FLOAT);
        SetShaderValue(g->visualShader, GetShaderLocation(g->visualShader, "reflectance"),
                       &source->reflectance, SHADER_UNIFORM_FLOAT);
        SetShaderValue(g->visualShader, GetShaderLocation(g->visualShader, "emissive"),
                       source->emissive, SHADER_UNIFORM_VEC3);
        DrawMeshInstanced(g->visuals[i].mesh, g->visuals[i].material, &matrix, 1);
    }
}

static int backToFront(const void *left, const void *right) {
    float a = ((const TransparentDraw *)left)->depth;
    float b = ((const TransparentDraw *)right)->depth;
    return (a < b) - (a > b);
}

static void drawInstances(TbGraphics *g, Batch *batch, const Matrix *transforms, size_t count) {
    Geometry *geometry = &g->geometries[batch->geometry];
    g->material.maps[MATERIAL_MAP_DIFFUSE].color = batch->color;
    if (geometry->nt) {
        DrawMeshInstanced(geometry->mesh, g->material, transforms, (int)count);
    }
    for (size_t j = 0; j < count; ++j) {
        for (size_t k = 0; k + 1 < geometry->nl; k += 2) {
            DrawLine3D(Vector3Transform(vec(geometry->lines[k]), transforms[j]),
                       Vector3Transform(vec(geometry->lines[k + 1]), transforms[j]), batch->color);
        }
    }
}

static void drawTransparent(TbGraphics *g) {
    if (!g->transparentCount) {
        return;
    }
    qsort(g->transparent, g->transparentCount, sizeof(*g->transparent), backToFront);
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (size_t i = 0; i < g->transparentCount; ++i) {
        TransparentDraw *draw = &g->transparent[i];
        if (draw->triangle) {
            DrawTriangle3D(draw->vertices[0], draw->vertices[1], draw->vertices[2], draw->color);
        } else {
            /* Instanced meshes draw immediately; flush queued triangles first to
             * preserve the sorted order across both rendering paths. */
            rlDrawRenderBatchActive();
            Batch *batch = &g->batches[draw->batch];
            drawInstances(g, batch, &batch->transforms[draw->instance], 1);
        }
    }
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
}

int tbGraphicsDraw(TbGraphics *g, Testbed *t, Camera3D camera, uint32_t debug, bool surfaces) {
    if (!t->world || t->error[0]) {
        return 0;
    }
    if (setjmp(t->failure)) {
        EndMode3D();
        rlEnableBackfaceCulling();
        return 0;
    }
    BeginMode3D(camera);
    rlDisableBackfaceCulling();
    for (size_t i = 0; i < g->entryCapacity; i++) {
        g->entries[i].soft = false;
        g->entries[i].seen = false;
    }
    for (size_t i = 0; i < g->batchCount; i++) {
        g->batches[i].count = 0;
    }
    g->transparentCount = 0;
    drawSoft(g, t, surfaces && t->collidersVisible, camera);
    if (surfaces) {
        drawVisualMeshes(g, t, camera);
    }
    size_t n = RAPIER_FN(ColliderHandles)(t->world, NULL, 0);
    TB(t, RAPIER_FN(LastStatus)());
    g->handles = grow(g->handles, &g->handlesCapacity, n, sizeof(*g->handles));
    n = RAPIER_FN(ColliderHandles)(t->world, g->handles, g->handlesCapacity);
    TB(t, RAPIER_FN(LastStatus)());
    for (size_t i = 0; i < n; i++) {
        RAPIER_TYPE(ColliderHandle) h = g->handles[i];
        Entry *e = entry(g, h);
        e->seen = true;
        if (!surfaces || !t->collidersVisible || e->soft) {
            continue;
        }
        TB(t, RAPIER_FN(Collider_ValidateHandle)(h));
        RAPIER_TYPE(Bool) enabled = RAPIER_FN(Collider_IsEnabled)(h);
        TB(t, RAPIER_FN(LastStatus)());
        if (!enabled) {
            continue;
        }
        uintptr_t identity = 0;
        identity = RAPIER_FN(Collider_ShapeIdentity)(h);
        TB(t, RAPIER_FN(LastStatus)());
        if (!e->valid || e->generation != h.generation || e->identity != identity) {
            (void)RAPIER_FN(FreeSharedShape)(e->shape);
            e->shape = NULL;
            e->shape = RAPIER_FN(Collider_CloneShape)(h);
            TB(t, RAPIER_FN(LastStatus)());
            e->geometry = geometry(g, t, e->shape);
            e->valid = true;
            e->generation = h.generation;
            e->identity = identity;
        }
        RAPIER_TYPE(Pose) p = RAPIER_FN(Collider_Position)(h);
        TB(t, RAPIER_FN(LastStatus)());
        instance(g, e->geometry, color(t, h), transform(p));
    }
    /* Opaque objects retain instancing. Sort translucent instances by depth so
     * sensors reveal objects behind them and do not occlude later sensors. */
    Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    for (size_t i = 0; i < g->batchCount; ++i) {
        Batch *batch = &g->batches[i];
        if (!batch->count) {
            continue;
        }
        if (batch->color.a == 255) {
            drawInstances(g, batch, batch->transforms, batch->count);
        } else {
            g->transparent = grow(g->transparent, &g->transparentCapacity,
                                  g->transparentCount + batch->count, sizeof(*g->transparent));
            for (size_t j = 0; j < batch->count; ++j) {
                Matrix transform = batch->transforms[j];
                Vector3 center = {transform.m12, transform.m13, transform.m14};
                g->transparent[g->transparentCount++] = (TransparentDraw){
                    .batch = i,
                    .instance = j,
                    .depth = Vector3DotProduct(Vector3Subtract(center, camera.position), forward)};
            }
        }
    }
    drawTransparent(g);
    if (debug) {
        size_t count = RAPIER_FN(DebugRender)(t->world, debug, NULL, 0);
        TB(t, RAPIER_FN(LastStatus)());
        g->debug = grow(g->debug, &g->debugCapacity, count, sizeof(*g->debug));
        count = RAPIER_FN(DebugRender)(t->world, debug, g->debug, g->debugCapacity);
        TB(t, RAPIER_FN(LastStatus)());
        for (size_t i = 0; i < count; i++) {
            RAPIER_TYPE(DebugLine) *line = &g->debug[i]; /* Rapier colors are HSLA, not RGBA. */
            float h = line->color[0], s = line->color[1], l = line->color[2];
            float v = l + s * fminf(l, 1 - l);
            Color c = ColorFromHSV(h, v > 0 ? 2 * (1 - l / v) : 0, v);
            c.a = (unsigned char)(255 * line->color[3]);
            DrawLine3D(vec(line->a), vec(line->b), c);
        }
    }
    for (size_t i = 0; i < t->lineCount; ++i) {
        const float *rgba = t->lines[i].rgba;
        Color color = {(unsigned char)(255 * rgba[0]), (unsigned char)(255 * rgba[1]),
                       (unsigned char)(255 * rgba[2]), (unsigned char)(255 * rgba[3])};
        DrawLine3D(vec(t->lines[i].a), vec(t->lines[i].b), color);
    }
    t->lineCount = 0;
    /* Reclaim removed/replaced geometry so fountains and shape edits have bounded caches. */
    for (size_t i = 0; i < g->geometryCount; i++) {
        g->geometries[i].live = false;
    }
    for (size_t i = 0; i < g->entryCapacity; i++) {
        Entry *e = &g->entries[i];
        if (!e->seen || e->soft) {
            (void)RAPIER_FN(FreeSharedShape)(e->shape);
            e->shape = NULL;
            e->valid = false;
        } else if (e->valid) {
            g->geometries[e->geometry].live = true;
        }
    }
    for (size_t i = 0; i < g->geometryCount; i++) {
        Geometry *m = &g->geometries[i];
        if (m->allocated && !m->live) {
            if (m->nt) {
                UnloadMesh(m->mesh);
            }
            free(m->triangles);
            free(m->lines);
            memset(m, 0, sizeof(*m));
        }
    }
    size_t kept = 0;
    for (size_t i = 0; i < g->batchCount; i++) {
        if (g->batches[i].count) {
            g->batches[kept++] = g->batches[i];
        } else {
            free(g->batches[i].transforms);
        }
    }
    g->batchCount = kept;
    /* EndMode3D flushes queued soft triangles. Keep both faces visible until
     * that batch has been drawn, then restore raylib's default for the UI. */
    EndMode3D();
    rlEnableBackfaceCulling();
    return 1;
}

void tbGraphicsFrameAll(Testbed *t, Camera3D *camera) {
    if (!t->world) {
        return;
    }
    if (setjmp(t->failure)) {
        return;
    }
    size_t n = RAPIER_FN(ColliderHandles)(t->world, NULL, 0);
    TB(t, RAPIER_FN(LastStatus)());
    RAPIER_TYPE(ColliderHandle) *h = malloc((n ? n : 1) * sizeof(*h));
    if (!h) {
        abort();
    }
    n = RAPIER_FN(ColliderHandles)(t->world, h, n);
    TB(t, RAPIER_FN(LastStatus)());
    Vector3 lo = {1e20f, 1e20f, 1e20f}, hi = {-1e20f, -1e20f, -1e20f};
    bool found = false;
    for (size_t i = 0; i < t->renderMeshCount; ++i) {
        const TbRenderMesh *mesh = &t->renderMeshes[i];

        RAPIER_TYPE(Pose) pose;
        TB(t, RAPIER_FN(RigidBody_ValidateHandle)(mesh->body));
        pose = RAPIER_FN(RigidBody_Position)(mesh->body);
        TB(t, RAPIER_FN(LastStatus)());
        Matrix matrix = MatrixMultiply(transform(mesh->localPose), transform(pose));
        for (size_t j = 0; j < mesh->vertexCount; ++j) {
            Vector3 point = Vector3Transform(vec(mesh->vertices[j]), matrix);
            lo = Vector3Min(lo, point);
            hi = Vector3Max(hi, point);
            found = true;
        }
    }
    for (size_t i = 0; t->collidersVisible && i < n; i++) {
        RAPIER_TYPE(Aabb) a;
        TB(t, RAPIER_FN(Collider_ValidateHandle)(h[i]));
        RAPIER_TYPE(RigidBodyHandle) p = RAPIER_FN(Collider_Parent)(h[i]);
        TB(t, RAPIER_FN(LastStatus)());
        if (p.index != UINT32_MAX) {
            RAPIER_TYPE(Bool) fixed;
            TB(t, RAPIER_FN(RigidBody_ValidateHandle)(p));
            fixed = RAPIER_FN(RigidBody_IsFixed)(p);
            TB(t, RAPIER_FN(LastStatus)());
            if (fixed) {
                continue;
            }
        }
        a = RAPIER_FN(Collider_ComputeAabb)(h[i]);
        TB(t, RAPIER_FN(LastStatus)());
        Vector3 l = vec(a.mins), u = vec(a.maxs);
        if (Vector3Distance(l, u) > 1e8f) {
            continue;
        }
        lo = Vector3Min(lo, l);
        hi = Vector3Max(hi, u);
        found = true;
    }
    free(h);
    if (!found) {
        return;
    }
    camera->target = Vector3Scale(Vector3Add(lo, hi), 0.5f);
    float size = fmaxf(Vector3Distance(lo, hi), 1.0f);
    Vector3 dir = Vector3Normalize(Vector3Subtract(camera->position, camera->target));
#if defined(RAPIER_DIM2)
    dir = (Vector3){0, 0, 1};
    float inverseAspect = (float)GetScreenHeight() / (float)GetScreenWidth();
    camera->fovy = fmaxf((hi.y - lo.y) * 1.2f, (hi.x - lo.x) * 1.2f * inverseAspect);
#endif
    camera->position = Vector3Add(camera->target, Vector3Scale(dir, size * 1.2f));
}
