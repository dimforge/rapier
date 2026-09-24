/* Exercise the actual C UI and ImGui input routing without a display or GPU. */
#define TB_UI_TESTING
#include "../gui.c"
#include <assert.h>

static void frame(UiState *ui, bool focusSearch) {
    igNewFrame();
    igSetNextWindowPos(UI2(0, 0), ImGuiCond_Always, UI2(0, 0));
    igSetNextWindowSize(UI2(600, 800), ImGuiCond_Always);
    igBegin("Input regression", NULL, ImGuiWindowFlags_NoSavedSettings);
    if (focusSearch) {
        igSetKeyboardFocusHere(0);
    }
    examplesUi(ui);
    igEnd();
    keyboardShortcuts(ui);
    igRender();
    /* A null renderer acknowledges atlas updates; no GPU upload is needed. */
    ImDrawData *draw = igGetDrawData();
    if (draw->Textures) {
        for (int i = 0; i < draw->Textures->Size; i++) {
            ImTextureData *texture = draw->Textures->Data[i];
            if (texture->Status == ImTextureStatus_WantCreate ||
                texture->Status == ImTextureStatus_WantUpdates) {
                ImTextureData_SetTexID(texture, 1);
                ImTextureData_SetStatus(texture, ImTextureStatus_OK);
            } else if (texture->Status == ImTextureStatus_WantDestroy) {
                ImTextureData_SetStatus(texture, ImTextureStatus_Destroyed);
            }
        }
    }
}

int main(void) {
    ImGuiContext *context = igCreateContext(NULL);
    assert(context);
    ImGuiIO *io = igGetIO_Nil();
    io->DisplaySize = UI2(1440, 900);
    io->DisplayFramebufferScale = UI2(1, 1);
    io->DeltaTime = 1.0f / 60;
    io->BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    igStyleColorsLight(NULL);
    assert(configureUi());
    UiState ui = {0};
    ui.running = true;
    ui.initialTab = "Examples";
    /* Navigation wraps around the available, filtered examples. */
    ui.selected = 0;
    int next = adjacentExample(&ui, 1);
    assert(next != 0 && !tbExamples[next].requires);
    ui.selected = next;
    assert(adjacentExample(&ui, -1) == 0);
    snprintf(ui.search, sizeof(ui.search), "no-such-example");
    assert(adjacentExample(&ui, 1) == next);
    ui.search[0] = 0;
#if defined(RAPIER_DIM3)
    Camera3D camera = {.position = {3, 2, 5}, .target = {1, 0, 0}, .up = {0, 1, 0}, .fovy = 45};
    float distance = Vector3Distance(camera.position, camera.target);
    Vector3 target = camera.target;
    orbitCamera(&camera, (Vector2){100, 50}, false, 0, 900);
    assert(Vector3Distance(camera.target, target) < .0001f);
    assert(fabsf(Vector3Distance(camera.position, camera.target) - distance) < .0001f);
    Vector3 offset = Vector3Subtract(camera.position, camera.target);
    orbitCamera(&camera, (Vector2){30, 20}, true, 0, 900);
    assert(Vector3Distance(camera.target, target) > .01f);
    assert(Vector3Distance(Vector3Subtract(camera.position, camera.target), offset) < .0001f);
    orbitCamera(&camera, (Vector2){0}, false, 1, 900);
    assert(Vector3Distance(camera.position, camera.target) < distance);
#endif
    frame(&ui, true);
    frame(&ui, false);
    assert(io->WantCaptureKeyboard);
    ImGuiIO_AddInputCharactersUTF8(io, "test");
    ImGuiKey keys[] = {ImGuiKey_T, ImGuiKey_S, ImGuiKey_R, ImGuiKey_F};
    for (size_t i = 0; i < TB_COUNT(keys); i++) {
        ImGuiIO_AddKeyEvent(io, keys[i], true);
        frame(&ui, false);
        assert(ui.running && !ui.stepOnce && !ui.restart && !ui.frameAll);
        ImGuiIO_AddKeyEvent(io, keys[i], false);
        frame(&ui, false);
    }
    assert(!strcmp(ui.search, "test"));
    assert(matches("Stress tests", ui.search));
    assert(!matches("Restitution", ui.search));
    /* Clicking the scene relinquishes capture; hotkeys work again. */
    ImGuiIO_AddMousePosEvent(io, 1000, 500);
    frame(&ui, false);
    ImGuiIO_AddMouseButtonEvent(io, 0, true);
    frame(&ui, false);
    ImGuiIO_AddMouseButtonEvent(io, 0, false);
    frame(&ui, false);
    frame(&ui, false);
    assert(!io->WantCaptureKeyboard && !io->WantCaptureMouse);
    ImGuiIO_AddKeyEvent(io, ImGuiKey_T, true);
    frame(&ui, false);
    assert(!ui.running);
    ImGuiIO_AddKeyEvent(io, ImGuiKey_T, false);
    frame(&ui, false);
    ImGuiIO_AddKeyEvent(io, ImGuiKey_S, true);
    frame(&ui, false);
    assert(ui.stepOnce);
    igDestroyContext(context);
    puts("Dear ImGui text capture, scene input, and simulation shortcuts passed");
    return 0;
}
