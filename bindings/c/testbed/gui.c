#include "testbed_internal.h"
#include "font_data.h"
#include "graphics.h"
#include "grab.h"
#include "raymath.h"
#define CIMGUI_DEFINE_ENUMS_AND_STRUCTS
#include "cimgui.h"
#include "rlImGui.h"
#include <ctype.h>
#include <float.h>
#include <inttypes.h>
#include <stdlib.h>

#define UI2(x, y) ((ImVec2_c){(float)(x), (float)(y)})
#define UI4(r, g, b, a) ((ImVec4_c){(r), (g), (b), (a)})
#define UI_HISTORY 180

typedef struct UiState {
    bool running, surfaces, debug[6];
    bool stepOnce, restart, frameAll, save, restore;
    int selected, next, frame, historyCount, historyOffset;
    char search[96];
    int threadInput;
    char threadError[256];
    const char *profile, *initialTab;
    double physicsMs, renderMs;
    int physicsSteps;
    float physicsHistory[UI_HISTORY], renderHistory[UI_HISTORY];
    RAPIER_TYPE(Bytes) *snapshot;
    uint64_t snapshotStep;
    double snapshotTime;
} UiState;

static const uint32_t debugModes[] = {RAPIER_CONST(DEBUG_COLLIDER_SHAPES),
                                      RAPIER_CONST(DEBUG_RIGID_BODY_AXES),
                                      RAPIER_CONST(DEBUG_CONTACTS),
                                      RAPIER_CONST(DEBUG_IMPULSE_JOINTS) |
                                          RAPIER_CONST(DEBUG_MULTIBODY_JOINTS),
                                      RAPIER_CONST(DEBUG_SOFT_BODIES),
                                      RAPIER_CONST(DEBUG_SOFT_BODY_STRESS)};
static const char *debugNames[] = {"Wireframes", "Body axes",        "Contacts",
                                   "Joints",     "Soft constraints", "Soft stress"};

static bool configureUi(void) {
    ImGuiIO *io = igGetIO_Nil();
    io->IniFilename = NULL;
    io->LogFilename = NULL;
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImFontConfig *config = ImFontConfig_ImFontConfig();
    // The embedded bytes remain alive for the entire ImGui context lifetime.
    config->FontDataOwnedByAtlas = false;
    io->FontDefault = ImFontAtlas_AddFontFromMemoryTTF(
        io->Fonts, (void *)tbFontData, (int)sizeof(tbFontData), 16.0f, config, NULL);
    ImFontConfig_destroy(config);
    if (!io->FontDefault) {
        return false;
    }
    // ImGui 1.92 rasterizes at the framebuffer density reported by rlImGui.
    ImGuiStyle *style = igGetStyle();
    style->FontSizeBase = 16;
    style->WindowPadding = UI2(16, 14);
    style->FramePadding = UI2(8, 5);
    style->ItemSpacing = UI2(8, 7);
    style->WindowRounding = 6;
    style->FrameRounding = 4;
    style->GrabRounding = 4;
    style->ChildRounding = 4;
    style->ScrollbarRounding = 6;
    style->Colors[ImGuiCol_Text] = UI4(0.235f, 0.227f, 0.204f, 1);
    style->Colors[ImGuiCol_TextDisabled] = UI4(0.52f, 0.52f, 0.49f, 1);
    style->Colors[ImGuiCol_WindowBg] = UI4(0.988f, 0.988f, 0.973f, 1);
    style->Colors[ImGuiCol_Border] = UI4(0.784f, 0.776f, 0.745f, 0.6f);
    style->Colors[ImGuiCol_FrameBg] = UI4(0.922f, 0.922f, 0.882f, 1);
    style->Colors[ImGuiCol_FrameBgHovered] = UI4(0.882f, 0.882f, 0.843f, 1);
    style->Colors[ImGuiCol_FrameBgActive] = UI4(0.843f, 0.843f, 0.804f, 1);
    style->Colors[ImGuiCol_Button] = UI4(0.922f, 0.922f, 0.882f, 1);
    style->Colors[ImGuiCol_ButtonHovered] = UI4(0.78f, 0.85f, 0.86f, 1);
    style->Colors[ImGuiCol_ButtonActive] = UI4(0.62f, 0.76f, 0.80f, 1);
    style->Colors[ImGuiCol_Header] = UI4(0.88f, 0.91f, 0.88f, 1);
    style->Colors[ImGuiCol_HeaderHovered] = UI4(0.78f, 0.85f, 0.86f, 1);
    style->Colors[ImGuiCol_HeaderActive] = UI4(0.62f, 0.76f, 0.80f, 1);
    style->Colors[ImGuiCol_Tab] = UI4(0.92f, 0.92f, 0.88f, 1);
    style->Colors[ImGuiCol_TabHovered] = UI4(0.78f, 0.85f, 0.86f, 1);
    style->Colors[ImGuiCol_TabSelected] = UI4(0.78f, 0.85f, 0.86f, 1);
    style->Colors[ImGuiCol_CheckMark] = UI4(0.322f, 0.510f, 0.588f, 1);
    style->Colors[ImGuiCol_SliderGrab] = UI4(0.322f, 0.510f, 0.588f, 1);
    style->Colors[ImGuiCol_PlotLines] = UI4(0.322f, 0.510f, 0.588f, 1);
    return true;
}

static bool setupUi(void) {
    rlImGuiSetup(false);
    return configureUi();
}

static void keyboardShortcuts(UiState *ui) {
    ImGuiIO *io = igGetIO_Nil();
    if (io->WantCaptureKeyboard || io->WantTextInput) {
        return;
    }
    if (igIsKeyPressed_Bool(ImGuiKey_T, false)) {
        ui->running = !ui->running;
    }
    if (igIsKeyPressed_Bool(ImGuiKey_S, false)) {
        ui->stepOnce = true;
    }
    if (igIsKeyPressed_Bool(ImGuiKey_R, false)) {
        ui->restart = true;
    }
    if (igIsKeyPressed_Bool(ImGuiKey_F, false)) {
        ui->frameAll = true;
    }
}

static void tooltip(const char *text) {
    if (igIsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && igBeginTooltip()) {
        igPushTextWrapPos(igGetFontSize() * 26);
        igTextUnformatted(text, NULL);
        igPopTextWrapPos();
        igEndTooltip();
    }
}

// UI setters report ordinary errors without unwinding an open ImGui window.
static void uiStatus(Testbed *t, RAPIER_TYPE(Status) status) {
    if (status != RAPIER_CONST(OK)) {
        snprintf(t->error, sizeof(t->error), "%s", RAPIER_FN(LastError)());
    }
}

static Camera3D cameraFor(Testbed *t) {
    Camera3D c = {0};
    c.position = (Vector3){t->eye[0], t->eye[1], t->eye[2]};
    c.target = (Vector3){t->target[0], t->target[1], t->target[2]};
    c.up = (Vector3){t->up[0], t->up[1], t->up[2]};
    c.fovy = 45;
    c.projection = CAMERA_PERSPECTIVE;
#if defined(RAPIER_DIM2)
    c.position = (Vector3){t->target[0], t->target[1], 100};
    c.target.z = 0;
    c.fovy = t->viewWidth * (float)GetScreenHeight() / (float)GetScreenWidth();
    c.projection = CAMERA_ORTHOGRAPHIC;
#endif
    return c;
}

static bool matches(const char *text, const char *query) {
    for (; *text; text++) {
        size_t i = 0;
        while (query[i] && text[i] &&
               tolower((unsigned char)text[i]) == tolower((unsigned char)query[i])) {
            i++;
        }
        if (!query[i]) {
            return true;
        }
    }
    return !*query;
}

static int abi(void) {
    RAPIER_TYPE(Status)
    s = RAPIER_FN(CheckAbi)(RAPIER_CONST(ABI_VERSION), RAPIER_CONST(DIMENSION),
                            sizeof(RAPIER_TYPE(Real)), sizeof(RAPIER_TYPE(Vector)),
                            sizeof(RAPIER_TYPE(Pose)), RAPIER_CONST(ABI_FEATURES));
    if (s) {
        fprintf(stderr, "ABI mismatch: %s\n", RAPIER_FN(LastError)());
    }
    return !s;
}

static void worldSnapshot(Testbed *t, RAPIER_TYPE(Bytes) **snapshot, int save, uint64_t *step,
                          double *time) {
    if (setjmp(t->failure)) {
        return;
    }
    if (save) {
        (void)RAPIER_FN(FreeBytes)(*snapshot);
        *snapshot = NULL;
        *snapshot = RAPIER_FN(SerializeWorld)(t->world);
        TB(t, RAPIER_FN(LastStatus)());
        *step = t->step;
        *time = t->time;
    } else if (*snapshot) {
        const uint8_t *data;
        size_t n;
        RAPIER_TYPE(World) *w = NULL;
        RAPIER_TYPE(ByteView) bytesDataResult = RAPIER_FN(Bytes_Data)(*snapshot);
        data = bytesDataResult.data;
        n = bytesDataResult.count;
        TB(t, RAPIER_FN(LastStatus)());
        w = RAPIER_FN(DeserializeWorld)(data, n);
        TB(t, RAPIER_FN(LastStatus)());
        TB(t, RAPIER_FN(FreeWorld)(t->world));
        t->world = w;
        t->step = *step;
        t->time = *time;
        tbRefreshWorld(t);
    }
}

static bool exampleMatches(const TbExample *e, const UiState *ui) {
    return !e->
                requires && (matches(e->name, ui->search) || matches(e->group, ui->search) ||
                             matches(e->id, ui->search));
}

/* Navigation follows the filtered list, wraps, and skips unavailable builds. */
static int adjacentExample(const UiState *ui, int direction) {
    int count = (int)tbExampleCount;
    for (int offset = 1; offset < count; ++offset) {
        int index = (ui->selected + direction * offset + count) % count;
        if (exampleMatches(&tbExamples[index], ui)) {
            return index;
        }
    }
    return ui->selected;
}

static void examplesUi(UiState *ui) {
    igSetNextItemWidth(-FLT_MIN);
    igInputTextWithHint("##search", "Search examples...", ui->search, sizeof(ui->search), 0, NULL,
                        NULL);
    for (size_t start = 0; start < tbExampleCount;) {
        size_t end = start + 1, count = 0;
        while (end < tbExampleCount && !strcmp(tbExamples[start].group, tbExamples[end].group)) {
            end++;
        }
        for (size_t i = start; i < end; i++) {
            count += exampleMatches(&tbExamples[i], ui);
        }
        if (count) {
            igPushID_Str(tbExamples[start].group);
            if (ui->search[0]) {
                igSetNextItemOpen(true, ImGuiCond_Always);
            } else {
                igSetNextItemOpen((size_t)ui->selected >= start && (size_t)ui->selected < end,
                                  ImGuiCond_Once);
            }
            char label[128];
            snprintf(label, sizeof(label), "%s (%zu)", tbExamples[start].group, count);
            if (igCollapsingHeader_TreeNodeFlags(label, 0)) {
                for (size_t i = start; i < end; i++) {
                    const TbExample *e = &tbExamples[i];
                    if (!exampleMatches(e, ui)) {
                        continue;
                    }
                    igPushID_Str(e->id);
                    igBeginDisabled(e->requires != NULL);
                    if (igSelectable_Bool(e->name, i == (size_t)ui->selected, 0, UI2(0, 0))) {
                        ui->next = (int)i;
                    }
                    igEndDisabled();
                    tooltip(e->requires ? e->requires : e->source);
                    igPopID();
                }
            }
            igPopID();
        }
        start = end;
    }
}

typedef size_t (*ParameterGet)(const RAPIER_TYPE(World) *);
typedef RAPIER_TYPE(Status) (*ParameterSet)(RAPIER_TYPE(World) *, size_t);

static void iterationSlider(Testbed *t, const char *name, ParameterGet get, ParameterSet set,
                            int min, int max) {
    size_t value = get(t->world);
    if (RAPIER_FN(LastStatus)() != RAPIER_CONST(OK)) {
        return;
    }
    int edit = value > (size_t)INT32_MAX ? INT32_MAX : (int)value;
    igTextUnformatted(name, NULL);
    igSetNextItemWidth(-FLT_MIN);
    igPushID_Str(name);
    if (igSliderInt("##value", &edit, min, max, "%d", ImGuiSliderFlags_AlwaysClamp)) {
        uiStatus(t, set(t->world, (size_t)edit));
    }
    igPopID();
}

static void threadingUi(Testbed *t, UiState *ui) {
    igText("SIMD: %u lanes", t->buildFeatures.simd_lanes);
    if (!t->buildFeatures.parallel) {
        igTextUnformatted("Parallelism: disabled in this build", NULL);
        igTextWrapped("Rebuild with RAPIER_ENABLE_PARALLEL=ON to select multiple workers.");
        return;
    }
    igText("Active workers: %zu", t->activeThreads);
    igSetNextItemWidth(145);
    igInputInt("Requested workers", &ui->threadInput, 1, 4, 0);
    tooltip("0 = automatic selection, 1 = one worker. Applies without resetting "
            "the simulation.");
    bool valid = ui->threadInput >= 0 && ui->threadInput <= TB_MAX_THREADS;
    igBeginDisabled(!valid || (size_t)ui->threadInput == t->requestedThreads);
    if (igButton("Apply threads", UI2(0, 0))) {
        RAPIER_TYPE(Status) status = tbSetThreads(t, (size_t)ui->threadInput);
        if (status != RAPIER_CONST(OK)) {
            snprintf(ui->threadError, sizeof(ui->threadError), "%s", RAPIER_FN(LastError)());
        } else {
            ui->threadError[0] = 0;
        }
    }
    igEndDisabled();
    igTextDisabled("0 = automatic | 1 = single worker");
    if (!valid) {
        igTextWrapped("Choose a worker count from 0 to %d.", TB_MAX_THREADS);
    }
    if (ui->threadError[0]) {
        igTextWrapped("%s", ui->threadError);
    }
}

static void settingsUi(Testbed *t, UiState *ui) {
    if (!t->world) {
        return;
    }
    if (igCollapsingHeader_TreeNodeFlags("Execution", ImGuiTreeNodeFlags_DefaultOpen)) {
        threadingUi(t, ui);
    }
    if (igCollapsingHeader_TreeNodeFlags("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        RAPIER_TYPE(Real) dt = RAPIER_FN(TimeStep)(t->world);
        uiStatus(t, RAPIER_FN(LastStatus)());
        float value = (float)dt;
        igTextUnformatted("Timestep (seconds)", NULL);
        igSetNextItemWidth(-FLT_MIN);
        if (igSliderFloat("##dt", &value, 0.001f, 0.05f, "%.4f", ImGuiSliderFlags_AlwaysClamp)) {
            uiStatus(t, RAPIER_FN(SetTimeStep)(t->world, (RAPIER_TYPE(Real))value));
        }
        RAPIER_TYPE(Vector) gravity = RAPIER_FN(Gravity)(t->world);
        uiStatus(t, RAPIER_FN(LastStatus)());
        igTextUnformatted("Gravity", NULL);
        igSetNextItemWidth(-FLT_MIN);
        RAPIER_TYPE(Real)
        g[] = {
            gravity.x,
            gravity.y,
#if defined(RAPIER_DIM3)
            gravity.z,
#endif
        };
        if (igInputScalarN("##gravity",
                           sizeof(RAPIER_TYPE(Real)) == 4 ? ImGuiDataType_Float
                                                          : ImGuiDataType_Double,
                           g, RAPIER_CONST(DIMENSION), NULL, NULL, "%.2f", 0)) {
            uiStatus(t, RAPIER_FN(SetGravity)(t->world, V(g[0], g[1], g[2])));
        }
    }
    if (igCollapsingHeader_TreeNodeFlags("Solver", 0)) {
        iterationSlider(t, "Solver iterations", RAPIER_FN(NumSolverIterations),
                        RAPIER_FN(SetNumSolverIterations), 1, 32);
        iterationSlider(t, "Internal PGS iterations", RAPIER_FN(NumInternalPgsIterations),
                        RAPIER_FN(SetNumInternalPgsIterations), 1, 32);
        iterationSlider(t, "Stabilization iterations",
                        RAPIER_FN(NumInternalStabilizationIterations),
                        RAPIER_FN(SetNumInternalStabilizationIterations), 0, 32);
        iterationSlider(t, "CCD substeps", RAPIER_FN(MaxCcdSubsteps), RAPIER_FN(SetMaxCcdSubsteps),
                        1, 32);
    }
    if (igCollapsingHeader_TreeNodeFlags("Example settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        igTextWrapped("Settings marked Live apply immediately; other changes require a restart.");
        bool noSleep = t->noSleep != 0;
        if (igCheckbox("Disable sleeping", &noSleep)) {
            t->noSleep = noSleep;
        }
        for (size_t i = 0; i < t->labelCount; ++i) {
            igText("%s %s", t->labels[i].name, t->labels[i].value);
        }
        if (!t->settingCount && !t->labelCount) {
            igTextDisabled("No example-specific parameters.");
        }
        for (size_t i = 0; i < t->settingCount; i++) {
            TbSetting *s = &t->settings[i];
            igPushID_Int((int)i);
            if (!s->choices && s->integer && s->min == 0 && s->max == 1) {
                bool value = s->value != 0;
                if (igCheckbox(s->name, &value)) {
                    s->value = value;
                }
                igPopID();
                continue;
            }
            igTextWrapped("%s%s", s->name, s->live ? " (Live)" : "");
            igSetNextItemWidth(-FLT_MIN);
            if (s->choices && s->choiceCount) {
                int selected = (int)s->value;
                if (igBeginCombo("##value", s->choices[selected], ImGuiComboFlags_HeightLarge)) {
                    for (size_t j = 0; j < s->choiceCount; ++j) {
                        if (igSelectable_Bool(s->choices[j], selected == (int)j, 0, UI2(0, 0))) {
                            s->value = (double)j;
                        }
                        if (selected == (int)j) {
                            igSetItemDefaultFocus();
                        }
                    }
                    igEndCombo();
                }
            } else if (s->integer && s->min == 0 && s->max == 1) {
                bool value = s->value != 0;
                if (igCheckbox("##value", &value)) {
                    s->value = value;
                }
            } else if (igSliderScalar("##value", ImGuiDataType_Double, &s->value, &s->min, &s->max,
                                      s->integer ? "%.0f" : "%.3g", ImGuiSliderFlags_AlwaysClamp) &&
                       s->integer) {
                s->value = round(s->value);
            }
            igPopID();
        }
    }
}

static void performanceUi(Testbed *t, UiState *ui) {
    igText("%.0f FPS", igGetIO_Nil()->Framerate);
    igText("Physics: %.2f ms/step", t->physicsStepMs);
    igText("Simulation: %.2f ms/frame (%d step%s)", ui->physicsMs, ui->physicsSteps,
           ui->physicsSteps == 1 ? "" : "s");
    igText("Draw CPU: %.2f ms/frame", ui->renderMs);
    igTextWrapped("Physics uses the same per-step engine counter as the Rust "
                  "testbed. Simulation includes the step and scene callbacks "
                  "in a rendered frame. Draw CPU excludes the UI and GPU execution.");
    if (ui->historyCount) {
        int offset = ui->historyCount == UI_HISTORY ? ui->historyOffset : 0;
        igPlotLines_FloatPtr("##physics", ui->physicsHistory, ui->historyCount, offset,
                             "Physics (ms/step)", 0, FLT_MAX, UI2(-1, 100), sizeof(float));
        igPlotLines_FloatPtr("##draw", ui->renderHistory, ui->historyCount, offset, "Draw CPU (ms)",
                             0, FLT_MAX, UI2(-1, 100), sizeof(float));
    }
    igSeparator();
    igText("Simulation time: %.2f s", t->time);
    igText("Steps: %" PRIu64, t->step);
}

static void sidebar(Testbed *t, UiState *ui, float width) {
    igSetNextWindowPos(UI2(0, 0), ImGuiCond_Always, UI2(0, 0));
    igSetNextWindowSize(UI2(width, GetScreenHeight()), ImGuiCond_Always);
    igBegin("Rapier Testbed", NULL,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings);
    igPushFont(NULL, 22);
    igTextUnformatted("Rapier C testbed", NULL);
    igPopFont();
    const bool release = !strcmp(ui->profile, "release");
    igTextColored(release ? UI4(0.21f, 0.45f, 0.34f, 1) : UI4(0.65f, 0.38f, 0.10f, 1),
                  "Physics: %s  |  %dD / f%d", release ? "Release" : "Debug",
                  RAPIER_CONST(DIMENSION), (int)sizeof(RAPIER_TYPE(Real)) * 8);
    tooltip("Cargo build profile reported by the loaded Rapier physics library, "
            "independent of the viewer's C/C++ build mode.");
    igText("SIMD: %u lanes  |  Parallel: %s", t->buildFeatures.simd_lanes,
           t->buildFeatures.parallel ? "enabled" : "disabled");
    if (t->buildFeatures.parallel && t->activeThreads) {
        igTextDisabled("Workers: %zu%s", t->activeThreads,
                       t->requestedThreads == 0 ? " (automatic)" : "");
    }
    tooltip("SIMD lane count and parallel support are reported by the loaded "
            "physics library. Configure workers in Settings > Execution.");
    igSeparator();
    int previous = adjacentExample(ui, -1), next = adjacentExample(ui, 1);
    float navigationWidth = (igGetContentRegionAvail().x - 8) / 2;
    igBeginDisabled(previous == ui->selected);
    if (igButton("Prev", UI2(navigationWidth, 0))) {
        ui->next = previous;
    }
    igEndDisabled();
    igSameLine(0, -1);
    igBeginDisabled(next == ui->selected);
    if (igButton("Next", UI2(navigationWidth, 0))) {
        ui->next = next;
    }
    igEndDisabled();
    float contentHeight = fmaxf(96, igGetContentRegionAvail().y - 196);
    if (igBeginTabBar("Main tabs", ImGuiTabBarFlags_None)) {
        const char *tabs[] = {"Examples", "Settings", "Performance", "Debug"};
        for (size_t i = 0; i < TB_COUNT(tabs); i++) {
            ImGuiTabItemFlags flags = (ui->frame == 0 && !strcmp(ui->initialTab, tabs[i]))
                                          ? ImGuiTabItemFlags_SetSelected
                                          : 0;
            if (igBeginTabItem(tabs[i], NULL, flags)) {
                if (igBeginChild_Str(tabs[i], UI2(0, contentHeight), 0, 0)) {
                    if (i == 0) {
                        examplesUi(ui);
                    } else if (i == 1) {
                        settingsUi(t, ui);
                    } else if (i == 2) {
                        performanceUi(t, ui);
                    } else {
                        igCheckbox("Surfaces", &ui->surfaces);
                        igSeparator();
                        for (size_t j = 0; j < TB_COUNT(debugNames); j++) {
                            igCheckbox(debugNames[j], &ui->debug[j]);
                        }
                        igSpacing();
                        igTextWrapped("Wireframes and overlays use Rapier's debug renderer.");
                    }
                }
                igEndChild();
                igEndTabItem();
            }
        }
        igEndTabBar();
    }
    igSeparator();
    float buttonWidth = (igGetContentRegionAvail().x - 16) / 3;
    if (igButton(ui->running ? "Pause [T]" : "Play [T]", UI2(buttonWidth, 0))) {
        ui->running = !ui->running;
    }
    igSameLine(0, -1);
    if (igButton("Step [S]", UI2(buttonWidth, 0))) {
        ui->stepOnce = true;
    }
    igSameLine(0, -1);
    if (igButton("Restart [R]", UI2(buttonWidth, 0))) {
        ui->restart = true;
    }
    if (igButton("Frame [F]", UI2(buttonWidth, 0))) {
        ui->frameAll = true;
    }
    igSameLine(0, -1);
    bool canSnapshot = t->snapshotSupported && !t->error[0];
    igBeginDisabled(!canSnapshot);
    if (igButton("Save", UI2(buttonWidth, 0))) {
        ui->save = true;
    }
    tooltip(canSnapshot ? "Save a physics snapshot."
                        : "This example has local simulation state that physics "
                          "snapshots do not serialize.");
    igSameLine(0, -1);
    igBeginDisabled(!ui->snapshot);
    if (igButton("Restore", UI2(buttonWidth, 0))) {
        ui->restore = true;
    }
    igEndDisabled();
    igEndDisabled();
    size_t nb = 0, nc = 0, ns = 0;
    if (t->world) {
        nb = RAPIER_FN(RigidBodyCount)(t->world);
        uiStatus(t, RAPIER_FN(LastStatus)());
        nc = RAPIER_FN(ColliderCount)(t->world);
        uiStatus(t, RAPIER_FN(LastStatus)());
        ns = RAPIER_FN(SoftBodyCount)(t->world);
        uiStatus(t, RAPIER_FN(LastStatus)());
    }
    igText("Bodies %zu  |  Colliders %zu  |  Soft %zu", nb, nc, ns);
    igText("Step %" PRIu64 "  |  Physics %.2f ms/step", t->step, t->physicsStepMs);
    igTextDisabled("Draw CPU %.2f ms  |  %d FPS", ui->renderMs, GetFPS());
    igEnd();
}

static void sceneOverlay(Testbed *t, float sidebarWidth) {
    igSetNextWindowPos(UI2(sidebarWidth + 18, 12), ImGuiCond_Always, UI2(0, 0));
    igSetNextWindowSize(UI2(fmaxf(100, (float)GetScreenWidth() - sidebarWidth - 36), 0), ImGuiCond_Always);
    igBegin("Scene", NULL,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
                ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_AlwaysAutoResize);
    igPushFont(NULL, 22);
    igTextUnformatted(t->example->name, NULL);
    igPopFont();
#if defined(RAPIER_DIM2)
    igTextDisabled("Left drag: grab   |   Right drag: pan   |   Wheel: zoom");
#else
    igTextDisabled("Left drag: grab   |   Right drag: orbit   |   Wheel: zoom");
    igTextDisabled("Shift + right drag / middle drag: pan");
#endif
    igTextDisabled("Arrows: move   |   Space: jump   |   Enter: action   |   Hold C: cut");
    if (t->error[0]) {
        igSpacing();
        igTextColored(UI4(0.70f, 0.18f, 0.14f, 1), "Scene error");
        igTextWrapped("%s", t->error);
    }
    igEnd();
}

#if defined(RAPIER_DIM3)
/* Orbit about the scene target, preserving distance and the example's up axis. */
static void orbitCamera(Camera3D *camera, Vector2 delta, bool pan, float wheel,
                        float viewportHeight) {
    Vector3 offset = Vector3Subtract(camera->position, camera->target);
    float distance = fmaxf(Vector3Length(offset), .001f);
    Vector3 up = Vector3Normalize(camera->up);
    Vector3 right = Vector3Normalize(Vector3CrossProduct(up, offset));
    if (Vector3LengthSqr(right) < 1.0e-8f) {
        right = Vector3Normalize(
            Vector3CrossProduct(fabsf(up.x) < .9f ? (Vector3){1, 0, 0} : (Vector3){0, 0, 1}, up));
    }
    if (pan) {
        Vector3 screenUp = Vector3Normalize(Vector3CrossProduct(offset, right));
        float scale = 2 * distance * tanf(camera->fovy * DEG2RAD * .5f) / viewportHeight;
        Vector3 shift = Vector3Add(Vector3Scale(right, -delta.x * scale),
                                   Vector3Scale(screenUp, delta.y * scale));
        camera->target = Vector3Add(camera->target, shift);
    } else {
        offset = Vector3RotateByAxisAngle(offset, up, -delta.x * .005f);
        Vector3 rotatedRight = Vector3CrossProduct(up, offset);
        if (Vector3LengthSqr(rotatedRight) > 1.0e-8f) {
            right = Vector3Normalize(rotatedRight);
        }
        float cosine = Vector3DotProduct(Vector3Normalize(offset), up);
        float polar = acosf(fmaxf(-1, fminf(1, cosine)));
        float nextPolar = fmaxf(.01f, fminf(PI - .01f, polar - delta.y * .005f));
        offset = Vector3RotateByAxisAngle(offset, right, nextPolar - polar);
    }
    float zoom = fmaxf(.001f, distance * expf(-wheel * .12f));
    camera->position = Vector3Add(camera->target, Vector3Scale(Vector3Normalize(offset), zoom));
}
#endif

typedef struct GuiViewer {
    UiState *ui;
    TbGraphics *graphics;
    TbGrab grab;
    Camera3D camera;
    int frames, sceneStarted;
    double physicsStart;
    uint64_t previousStep;
} GuiViewer;

static int renderFrame(Testbed *t, void *context) {
    GuiViewer *viewer = context;
    UiState *ui = viewer->ui;
    if (!viewer->sceneStarted) {
        viewer->sceneStarted = 1;
        viewer->grab = (TbGrab){0};
        for (size_t i = 0; i < TB_COUNT(debugModes); ++i) {
            ui->debug[i] = (t->initialDebug & debugModes[i]) != 0;
        }
        if (!t->preserveCamera || !ui->frame) {
            viewer->camera = cameraFor(t);
        }
        if (t->frameAll) {
            tbGraphicsFrameAll(t, &viewer->camera);
        }
        viewer->previousStep = 0;
        ui->physicsSteps = 0;
        ui->physicsMs = 0;
        TraceLog(LOG_INFO, "UI: SIMD %u lanes; parallel %s; workers %zu (requested %zu)",
                 t->buildFeatures.simd_lanes, t->buildFeatures.parallel ? "enabled" : "disabled",
                 t->activeThreads, t->requestedThreads);
    } else {
        ui->physicsSteps = t->step > viewer->previousStep;
        ui->physicsMs = ui->physicsSteps ? (GetTime() - viewer->physicsStart) * 1000 : 0;
    }
    viewer->previousStep = t->step;
    if (WindowShouldClose() || (viewer->frames && ui->frame >= viewer->frames)) {
        uiStatus(t, tbGrabRelease(t, &viewer->grab));
        return 0;
    }
    Camera3D camera = viewer->camera;
    TbGraphics *g = viewer->graphics;

    ui->stepOnce = ui->restart = ui->frameAll = ui->save = ui->restore = false;
    // Build the UI first so widgets can consume mouse and keyboard input.
    rlImGuiBegin();
    const float sidebarWidth = 390;
    sidebar(t, ui, sidebarWidth);
    sceneOverlay(t, sidebarWidth);
    ImGuiIO *io = igGetIO_Nil();
    bool captureKeyboard = io->WantCaptureKeyboard || io->WantTextInput;
    keyboardShortcuts(ui);
    Vector2 mouse = GetMousePosition();
    bool captureMouse = io->WantCaptureMouse || mouse.x < sidebarWidth;
    if (!captureMouse && !viewer->grab.active) {
#if defined(RAPIER_DIM2)
        float wheel = GetMouseWheelMove();
        camera.fovy = fmaxf(0.05f, camera.fovy * expf(-wheel * 0.12f));
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            float scale = camera.fovy / (float)GetScreenHeight();
            camera.position.x -= d.x * scale;
            camera.target.x -= d.x * scale;
            camera.position.y += d.y * scale;
            camera.target.y += d.y * scale;
        }
#else
        bool pan = IsMouseButtonDown(MOUSE_BUTTON_MIDDLE) || IsKeyDown(KEY_LEFT_SHIFT) ||
                   IsKeyDown(KEY_RIGHT_SHIFT);
        Vector2 delta =
            (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE))
                ? GetMouseDelta()
                : (Vector2){0};
        orbitCamera(&camera, delta, pan, GetMouseWheelMove(), (float)GetScreenHeight());
#endif
    }
    t->inputDirection = captureKeyboard
                            ? V(0, 0, 0)
                            : V((IsKeyDown(KEY_RIGHT) ? 1 : 0) - (IsKeyDown(KEY_LEFT) ? 1 : 0),
                                (IsKeyDown(KEY_UP) ? 1 : 0) - (IsKeyDown(KEY_DOWN) ? 1 : 0), 0);
    t->jump = !captureKeyboard && IsKeyDown(KEY_SPACE);
    t->boost = !captureKeyboard && IsKeyDown(KEY_RIGHT_SHIFT);
    t->descend = !captureKeyboard && IsKeyDown(KEY_RIGHT_CONTROL);
    t->slow = !captureKeyboard && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT));
#if defined(RAPIER_DIM3)
    Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera.up));
    t->cameraRight = V(right.x, 0, right.z);
    t->cameraForward = V(forward.x, 0, forward.z);
#endif
    t->action = !captureKeyboard && IsKeyPressed(KEY_ENTER);
    t->cutting = !captureKeyboard && IsKeyDown(KEY_C);
    t->cursorValid = 0;
    t->rayValid = 0;
    t->removeVoxel = !captureKeyboard && IsKeyDown(KEY_LEFT_SHIFT);
#if defined(RAPIER_DIM2)
    if (!captureMouse) {
        Ray ray = GetScreenToWorldRay(mouse, camera);
        t->rayValid = 1;
        t->rayOrigin = V(ray.position.x, ray.position.y, ray.position.z);
        t->rayDirection = V(ray.direction.x, ray.direction.y, ray.direction.z);
        if (fabsf(ray.direction.z) > 1.0e-6f) {
            float distance = -ray.position.z / ray.direction.z;
            t->cursor = V(ray.position.x + distance * ray.direction.x,
                          ray.position.y + distance * ray.direction.y, 0);
            t->cursorValid = 1;
        }
    }
#else
    if (!captureMouse) {
        Ray ray = GetScreenToWorldRay(mouse, camera);
        t->rayValid = 1;
        t->rayOrigin = V(ray.position.x, ray.position.y, ray.position.z);
        t->rayDirection = V(ray.direction.x, ray.direction.y, ray.direction.z);
        Vector3 normal = Vector3Normalize(Vector3Subtract(camera.position, camera.target));
        float denominator = Vector3DotProduct(ray.direction, normal);
        if (fabsf(denominator) > 1.0e-6f) {
            float distance = -Vector3DotProduct(ray.position, normal) / denominator;
            if (distance >= 0) {
                Vector3 point = Vector3Add(ray.position, Vector3Scale(ray.direction, distance));
                t->cursor = V(point.x, point.y, point.z);
                t->cursorValid = 1;
            }
        }
    }
#endif
    bool transition = ui->next != ui->selected || ui->restart;
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || transition || ui->save || ui->restore ||
        !IsWindowFocused()) {
        uiStatus(t, tbGrabRelease(t, &viewer->grab));
    } else if (!captureMouse && !t->error[0]) {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            RAPIER_TYPE(Real) radius = (RAPIER_TYPE(Real))(camera.fovy * 8 / (float)GetScreenHeight());
            uiStatus(t, tbGrabBegin(t, &viewer->grab, radius));
        }
        Vector3 direction = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
        uiStatus(t, tbGrabUpdate(t, &viewer->grab, V(direction.x, direction.y, direction.z)));
    }
    tbGrabDrawCue(t, &viewer->grab);
    if (ui->save) {
        worldSnapshot(t, &ui->snapshot, 1, &ui->snapshotStep, &ui->snapshotTime);
    }
    if (ui->restore) {
        worldSnapshot(t, &ui->snapshot, 0, &ui->snapshotStep, &ui->snapshotTime);
        tbGraphicsFree(g);
        g = tbGraphicsNew();
    }
    if (ui->frameAll) {
        tbGraphicsFrameAll(t, &camera);
    }
    if (t->error[0]) {
        ui->running = false;
    }
    t->simulating = !transition && !t->error[0] && (ui->running || ui->stepOnce);
    ui->stepOnce = false;
    double start;
    BeginDrawing();
    ClearBackground((Color){250, 250, 245, 255});
    uint32_t debug = 0;
    for (size_t i = 0; i < TB_COUNT(debugModes); i++) {
        if (ui->debug[i]) {
            debug |= debugModes[i];
        }
    }
    start = GetTime();
    tbGraphicsDraw(g, t, camera, debug, ui->surfaces);
    ui->renderMs = (GetTime() - start) * 1000;
    rlImGuiEnd();
    EndDrawing();
    ui->physicsHistory[ui->historyOffset] = (float)(ui->physicsSteps ? t->physicsStepMs : 0);
    ui->renderHistory[ui->historyOffset] = (float)ui->renderMs;
    ui->historyOffset = (ui->historyOffset + 1) % UI_HISTORY;
    if (ui->historyCount < UI_HISTORY) {
        ui->historyCount++;
    }
    ui->frame++;

    viewer->graphics = g;
    viewer->camera = camera;
    viewer->physicsStart = GetTime();
    return !transition;
}

int tbGui(int argc, char **argv) {
    if (!abi()) {
        return 1;
    }
    int initial = 0, noSleep = 0, frames = 0;
    size_t threads = 0;
    const char *initialTab = "Examples";
    const char *screenshot = NULL, *assets = TB_ASSET_ROOT;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--headless") || !strcmp(argv[i], "--list") ||
            !strcmp(argv[i], "--all")) {
            return tbHeadless(argc, argv);
        }
        if (!strcmp(argv[i], "--example") && i + 1 < argc) {
            const char *name = argv[++i];
            initial = -1;
            for (size_t j = 0; j < tbExampleCount; j++) {
                if (!strcmp(name, tbExamples[j].id)) {
                    initial = (int)j;
                }
            }
            if (initial < 0) {
                fprintf(stderr, "Unknown example: %s\n", name);
                return 2;
            }
        } else if (!strcmp(argv[i], "--no-sleep")) {
            noSleep = 1;
        } else if (!strcmp(argv[i], "--frames") && i + 1 < argc) {
            frames = atoi(argv[++i]);
            if (frames <= 0) {
                return 2;
            }
        } else if (!strcmp(argv[i], "--screenshot") && i + 1 < argc) {
            screenshot = argv[++i];
        } else if (!strcmp(argv[i], "--ui-tab") && i + 1 < argc) {
            initialTab = argv[++i];
            if (strcmp(initialTab, "Examples") && strcmp(initialTab, "Settings") &&
                strcmp(initialTab, "Performance") && strcmp(initialTab, "Debug")) {
                fputs("--ui-tab expects Examples, Settings, Performance, or Debug\n", stderr);
                return 2;
            }
        } else if (!strcmp(argv[i], "--threads") && i + 1 < argc) {
            if (!tbParseThreads(argv[++i], &threads)) {
                fprintf(stderr, "--threads expects an integer from 0 (automatic) to %d\n",
                        TB_MAX_THREADS);
                return 2;
            }
        } else if (!strcmp(argv[i], "--assets") && i + 1 < argc) {
            assets = argv[++i];
        } else {
            printf("Usage: %s [--example ID] [--no-sleep] [--frames N --screenshot "
                   "FILE] [--assets PATH] [--ui-tab TAB] [--threads N]\n       "
                   "--headless [--all | "
                   "--example ID] "
                   "[--steps N]\n",
                   argv[0]);
            return !strcmp(argv[i], "--help") ? 0 : 2;
        }
    }

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(1440, 900, "Rapier C testbed - Dear ImGui");
    if (!IsWindowReady()) {
        fputs("Cannot create a graphics window. Use rapier_testbed_headless "
              "without a display.\n",
              stderr);
        return 1;
    }
    SetWindowMinSize(850, 500);
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);
    if (!setupUi()) {
        rlImGuiShutdown();
        CloseWindow();
        return 1;
    }
    UiState ui = {0};
    ui.running = true;
    ui.surfaces = true;
    ui.selected = initial;
    ui.next = initial;
    ui.initialTab = initialTab;
    ui.threadInput = (int)threads;
    ui.profile = RAPIER_FN(BuildProfile)();
    TraceLog(LOG_INFO, "UI: Dear ImGui %s; physics compiled in %s mode", igGetVersion(),
             ui.profile);
    Testbed *t = calloc(1, sizeof(*t));
    if (!t) {
        abort();
    }
    t->noSleep = noSleep;
    t->requestedThreads = threads;
    t->assetRoot = assets;
    GuiViewer viewer = {.ui = &ui, .frames = frames};
    t->renderFrame = renderFrame;
    t->viewer = &viewer;
    int preserve = 0;
    while (!WindowShouldClose() && (!frames || ui.frame < frames)) {
        viewer.graphics = tbGraphicsNew();
        viewer.sceneStarted = 0;
        ui.historyCount = ui.historyOffset = 0;
        (void)tbRun(t, &tbExamples[ui.selected], preserve);
        if (t->error[0]) {
            ui.running = false;
            /* Keep the error visible and allow selection of another example. */
            while (renderFrame(t, &viewer)) {
            }
        }
        tbGraphicsFree(viewer.graphics);
        viewer.graphics = NULL;
        preserve = ui.next == ui.selected;
        ui.selected = ui.next;
        (void)RAPIER_FN(FreeBytes)(ui.snapshot);
        ui.snapshot = NULL;
    }
    if (screenshot) {
        Image capture = LoadImageFromScreen();
        if (!ExportImage(capture, screenshot)) {
            snprintf(t->error, sizeof(t->error), "Cannot save screenshot: %s", screenshot);
        }
        UnloadImage(capture);
    }
    int result = t->error[0] ? 1 : 0;
    TraceLog(LOG_INFO, "UI: completed %d frames, physics step %llu, profile %s", ui.frame,
             (unsigned long long)t->step, ui.profile);
    tbDestroy(t);
    free(t);
    (void)RAPIER_FN(FreeBytes)(ui.snapshot);
    rlImGuiShutdown();
    CloseWindow();
    return result;
}
#ifndef TB_UI_TESTING
int main(int argc, char **argv) {
    return tbGui(argc, argv);
}
#endif
