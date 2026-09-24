# The headless runner shares exactly the same C scene sources and callbacks.
file(GLOB_RECURSE TB_SCENES CONFIGURE_DEPENDS "examples${RAPIER_DIMENSION}d/*.c")
add_library(rapier_testbed_core STATIC testbed.c headless.c grab.c "registry${RAPIER_DIMENSION}.c" ${TB_SCENES})
target_link_libraries(rapier_testbed_core PUBLIC Rapier::rapier)
target_include_directories(rapier_testbed_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(rapier_testbed_core PUBLIC c_std_11)
target_compile_definitions(rapier_testbed_core PUBLIC TB_ASSET_ROOT="${RAPIER_ROOT}/assets")
if(NOT MSVC)
  target_link_libraries(rapier_testbed_core PUBLIC m)
  target_compile_options(rapier_testbed_core PRIVATE -Wall -Wextra)
endif()
rapier_executable(rapier_testbed_headless headless_main.c)
target_link_libraries(rapier_testbed_headless PRIVATE rapier_testbed_core)
# Explicit opt-in target; performance checks are not pass/fail timing tests.
rapier_executable(rapier_testbed_step_benchmark tools/step_benchmark.c)
set_target_properties(rapier_testbed_step_benchmark PROPERTIES EXCLUDE_FROM_ALL TRUE)
target_link_libraries(rapier_testbed_step_benchmark PRIVATE rapier_testbed_core)
if(RAPIER_TESTBED_GRAPHICS)
  # Normal variables stay in this directory: don't alter the parent project's options.
  set(BUILD_EXAMPLES OFF)
  set(BUILD_SHARED_LIBS OFF)
  # raylib 6.0's custom CMake flag parser treats even `#define FLAG 0` as ON.
  # Use config.h defaults and explicit numeric overrides to retain automatic
  # buffer swaps, input polling, and frame timing in EndDrawing().
  set(CUSTOMIZE_BUILD OFF)
  set(SUPPORT_MODULE_RAUDIO OFF CACHE BOOL "" FORCE)
  set(USE_AUDIO OFF CACHE BOOL "" FORCE)
  set(USE_EXTERNAL_GLFW OFF CACHE STRING "" FORCE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
  set(OPENGL_VERSION "3.3" CACHE STRING "" FORCE)
  add_subdirectory(vendor/raylib EXCLUDE_FROM_ALL)
  target_compile_definitions(raylib PRIVATE
    SUPPORT_MODULE_RAUDIO=0 SUPPORT_CUSTOM_FRAME_CONTROL=0 SUPPORT_BUSY_WAIT_LOOP=0)
  # Version-matched C wrappers and renderer; the testbed itself stays C11.
  set(TB_IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/cimgui/imgui")
  add_library(rapier_testbed_imgui STATIC
    vendor/cimgui/cimgui.cpp vendor/rlImGui/rlImGui.cpp
    "${TB_IMGUI_DIR}/imgui.cpp" "${TB_IMGUI_DIR}/imgui_draw.cpp"
    "${TB_IMGUI_DIR}/imgui_widgets.cpp" "${TB_IMGUI_DIR}/imgui_tables.cpp"
    "${TB_IMGUI_DIR}/imgui_demo.cpp")
  target_compile_features(rapier_testbed_imgui PRIVATE cxx_std_17)
  target_compile_definitions(rapier_testbed_imgui PUBLIC NO_FONT_AWESOME CIMGUI_NO_EXPORT)
  target_include_directories(rapier_testbed_imgui SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/cimgui"
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/rlImGui" "${TB_IMGUI_DIR}")
  target_link_libraries(rapier_testbed_imgui PUBLIC raylib)
  # Embed the font with CMake alone: no runtime asset path, font installation,
  # generator dependency, or network access is required by consumers.
  set(TB_FONT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/fira/FiraSans-Regular.ttf")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${TB_FONT_FILE}")
  file(READ "${TB_FONT_FILE}" TB_FONT_HEX HEX)
  string(REGEX REPLACE "(..)" "0x\\1," TB_FONT_BYTES "${TB_FONT_HEX}")
  # Bound source line length for compilers such as MSVC.
  string(REPEAT "0x[0-9a-f][0-9a-f]," 16 TB_FONT_ROW)
  string(REGEX REPLACE "(${TB_FONT_ROW})" "\\1\n" TB_FONT_BYTES "${TB_FONT_BYTES}")
  configure_file(font_data.h.in "${CMAKE_CURRENT_BINARY_DIR}/font_data.h" @ONLY)
  unset(TB_FONT_HEX)
  unset(TB_FONT_BYTES)
  unset(TB_FONT_ROW)
  rapier_executable(rapier_testbed gui.c)
  target_include_directories(rapier_testbed PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
  add_custom_command(TARGET rapier_testbed POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/NOTICES.txt"
      "$<TARGET_FILE_DIR:rapier_testbed>/THIRD_PARTY_NOTICES.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/licenses"
      "$<TARGET_FILE_DIR:rapier_testbed>/licenses"
    COMMAND "${CMAKE_COMMAND}" -E make_directory
      "$<TARGET_FILE_DIR:rapier_testbed>/licenses/glfw-mingw"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/src/external/glfw/deps/mingw/dinput.h"
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/src/external/glfw/deps/mingw/xinput.h"
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/src/external/glfw/deps/mingw/_mingw_dxhelper.h"
      "$<TARGET_FILE_DIR:rapier_testbed>/licenses/glfw-mingw"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/LICENSE"
      "$<TARGET_FILE_DIR:rapier_testbed>/raylib-LICENSE.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/src/external/glfw/LICENSE.md"
      "$<TARGET_FILE_DIR:rapier_testbed>/GLFW-LICENSE.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/fira/LICENSE"
      "$<TARGET_FILE_DIR:rapier_testbed>/FiraSans-LICENSE.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/cimgui/LICENSE"
      "$<TARGET_FILE_DIR:rapier_testbed>/cimgui-LICENSE.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/cimgui/imgui/LICENSE.txt"
      "$<TARGET_FILE_DIR:rapier_testbed>/DearImGui-LICENSE.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/rlImGui/LICENSE"
      "$<TARGET_FILE_DIR:rapier_testbed>/rlImGui-LICENSE.txt"
    VERBATIM)
  target_sources(rapier_testbed PRIVATE graphics.c)
  target_link_libraries(rapier_testbed PRIVATE rapier_testbed_core rapier_testbed_imgui)
  if(RAPIER_BUILD_TESTS)
    rapier_executable(rapier_testbed_ui_test tests/ui.c)
    target_sources(rapier_testbed_ui_test PRIVATE graphics.c)
    target_include_directories(rapier_testbed_ui_test PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
    target_link_libraries(rapier_testbed_ui_test PRIVATE rapier_testbed_core rapier_testbed_imgui)
    add_test(NAME testbed_ui_input COMMAND rapier_testbed_ui_test)
    rapier_executable(rapier_testbed_soft_render_test tests/soft_render.c)
    target_link_libraries(rapier_testbed_soft_render_test PRIVATE rapier_testbed_core raylib)
    add_test(NAME testbed_soft_render COMMAND rapier_testbed_soft_render_test)
    set_tests_properties(testbed_soft_render PROPERTIES TIMEOUT 30)
  endif()
endif()
if(RAPIER_BUILD_TESTS)
  rapier_executable(rapier_testbed_grab_test tests/grab.c)
  target_link_libraries(rapier_testbed_grab_test PRIVATE rapier_testbed_core)
  add_test(NAME testbed_mouse_grab COMMAND rapier_testbed_grab_test)
  rapier_executable(rapier_testbed_interactive_test tests/interactive.c)
  target_link_libraries(rapier_testbed_interactive_test PRIVATE rapier_testbed_core)
  add_test(NAME testbed_interactions COMMAND rapier_testbed_interactive_test)
  rapier_executable(rapier_testbed_threading_test tests/threading.c)
  target_link_libraries(rapier_testbed_threading_test PRIVATE rapier_testbed_core)
  add_test(NAME testbed_threading COMMAND rapier_testbed_threading_test)
  rapier_executable(rapier_testbed_error_test tests/errors.c)
  target_link_libraries(rapier_testbed_error_test PRIVATE rapier_testbed_core)
  add_test(NAME testbed_error_reporting COMMAND "${CMAKE_COMMAND}"
    "-DEXECUTABLE=$<TARGET_FILE:rapier_testbed_error_test>" -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/errors.cmake")
  add_test(NAME testbed_rigid COMMAND rapier_testbed_headless --example "restitution${RAPIER_DIMENSION}" --steps 120 --no-sleep)
  add_test(NAME testbed_dynamic COMMAND rapier_testbed_headless --example $<IF:$<STREQUAL:${RAPIER_DIMENSION},2>,add_remove2,fountain3> --steps 120 --no-sleep)
endif()
