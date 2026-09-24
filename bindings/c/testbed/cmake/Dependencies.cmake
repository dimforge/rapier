# Fetch sources only: the testbed controls which modules and targets are built.
# SOURCE_SUBDIR deliberately names a nonexistent directory to avoid configuring
# the upstream examples, generators, and independent CMake projects.
if(CMAKE_VERSION VERSION_LESS 3.25)
  message(FATAL_ERROR "The graphical testbed requires CMake 3.25 or newer (raylib 6.0).")
endif()
include(FetchContent)
FetchContent_Declare(raylib
  URL https://codeload.github.com/raysan5/raylib/tar.gz/dbc56a87da87d973a9c5baa4e7438a9d20121d28
  URL_HASH SHA256=81b06ce7c19cf3b634b0271c23c361ba6ad8bf45fb8b036abbfeb4260ec1e126
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE TLS_VERIFY TRUE
  SOURCE_SUBDIR rapier-no-cmake)
FetchContent_Declare(cimgui
  URL https://codeload.github.com/cimgui/cimgui/tar.gz/d3f0c2f4a7d4d116ef908295b971a36bdfdafe27
  URL_HASH SHA256=39bae7634da15173b6dd9b9aea6719148d9509c0cd625b6f81cab77720656ca9
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE TLS_VERIFY TRUE
  SOURCE_SUBDIR rapier-no-cmake)
# This is the imgui submodule revision recorded by the pinned cimgui commit.
FetchContent_Declare(imgui
  URL https://codeload.github.com/ocornut/imgui/tar.gz/dac07199cfd761113d966eb8ad739254e10df2fe
  URL_HASH SHA256=d95368fadc5a1665fc1b3f198d4c9ff026e8bd8edcc7bad5d82fa12cf379c5f3
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE TLS_VERIFY TRUE
  SOURCE_SUBDIR rapier-no-cmake)
FetchContent_Declare(rlimgui
  URL https://codeload.github.com/raylib-extras/rlImGui/tar.gz/3bc5731c4216bb8caa67fbea24aa85ce80d57ccb
  URL_HASH SHA256=2198c4eb4c0a2b8efc2204da046ef014a2c7b736e1483039badee6d19a7ed35a
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE TLS_VERIFY TRUE
  SOURCE_SUBDIR rapier-no-cmake)
# Download just the UI font, not Mozilla's complete font collection.
FetchContent_Declare(fira_font
  URL https://raw.githubusercontent.com/mozilla/Fira/fd8c8c0a3d353cd99e8ca1662942d165e6961407/ttf/FiraSans-Regular.ttf
  URL_HASH SHA256=a389cef71891df1232370fcebd7cfde5f74e741967070399adc91fd069b2094b
  DOWNLOAD_NO_EXTRACT TRUE TLS_VERIFY TRUE)
FetchContent_MakeAvailable(raylib cimgui imgui rlimgui fira_font)

# cimgui uses "./imgui/..." includes, but GitHub archives omit its submodule.
# Reproduce that header layout in the build directory without modifying either
# downloaded sources or user-supplied FETCHCONTENT_SOURCE_DIR_* checkouts.
set(TB_CIMGUI_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/cimgui-include")
foreach(header imgui.h imgui_internal.h imconfig.h imstb_rectpack.h imstb_textedit.h imstb_truetype.h)
  configure_file("${imgui_SOURCE_DIR}/${header}"
    "${TB_CIMGUI_INCLUDE_DIR}/imgui/${header}" COPYONLY)
endforeach()
