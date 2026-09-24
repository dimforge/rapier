# Testbed dependencies

The graphical viewer uses CMake `FetchContent` to download these exact revisions
at **configure time**, then compiles them with the testbed. Archive/file SHA-256
hashes are pinned in [cmake/Dependencies.cmake](cmake/Dependencies.cmake).
Sources are stored under the build directory's `_deps/`; nothing is downloaded
into the repository. No Git checkout, package installation, or binding generator
is needed for these dependencies.

| Dependency | Revision | License |
| --- | --- | --- |
| [raylib 6.0](https://github.com/raysan5/raylib) | `dbc56a87da87d973a9c5baa4e7438a9d20121d28` | zlib |
| [Dear ImGui 1.92.7](https://github.com/ocornut/imgui) | `dac07199cfd761113d966eb8ad739254e10df2fe` | MIT |
| [cimgui 1.92.7](https://github.com/cimgui/cimgui) | `d3f0c2f4a7d4d116ef908295b971a36bdfdafe27` | MIT |
| [rlImGui Raylib_6_0](https://github.com/raylib-extras/rlImGui) | `3bc5731c4216bb8caa67fbea24aa85ce80d57ccb` | zlib |
| [Fira Sans Regular](https://github.com/mozilla/Fira) | `fd8c8c0a3d353cd99e8ca1662942d165e6961407` | SIL OFL 1.1 |

Dear ImGui matches cimgui's submodule revision. Only the regular Fira font is
downloaded, then embedded in the executable. There is no runtime network or font
installation requirement. The UI remains C11; ImGui and its wrappers use C++17.

Raylib uses desktop GLFW and OpenGL 3.3 (macOS, Windows, Linux/X11). Audio, model
importers, mesh generators, raylib font-file loaders, compression helpers, and
gesture detection are disabled. Full upstream archives are downloaded, but their
examples, generators, and unused backends are not built. Texture loading,
screenshot export, mesh instancing, and the default font/white texture remain.

## Offline builds

An existing populated build directory can be reconfigured with
`-DFETCHCONTENT_FULLY_DISCONNECTED=ON`. This requires the sources to have already
been downloaded; it does not bootstrap an empty build directory.

For a new offline build, provide extracted sources at the pinned revisions:

```sh
cmake -S c -B build/c3 -DRAPIER_BUILD_TESTBED=ON \
  -DFETCHCONTENT_SOURCE_DIR_RAYLIB=/path/to/raylib \
  -DFETCHCONTENT_SOURCE_DIR_CIMGUI=/path/to/cimgui \
  -DFETCHCONTENT_SOURCE_DIR_IMGUI=/path/to/imgui \
  -DFETCHCONTENT_SOURCE_DIR_RLIMGUI=/path/to/rlImGui \
  -DFETCHCONTENT_SOURCE_DIR_FIRA_FONT=/path/to/font-directory
```

The font directory must contain `FiraSans-Regular.ttf`. Source overrides are
used as-is; archive hashes are checked only for downloaded dependencies. CMake
does not modify the supplied source trees. Cargo dependencies must also be
available separately for an offline physics build.

`RAPIER_BUILD_TESTBED=OFF` or `RAPIER_TESTBED_GRAPHICS=OFF` skips all five downloads.
See the [FetchContent documentation](https://cmake.org/cmake/help/latest/module/FetchContent.html)
for its cache and source override options.

## Redistribution

Ship the license files, `THIRD_PARTY_NOTICES.txt`, and `licenses/` directory copied
beside the viewer. Downloading instead of vendoring does not change the licenses.
The collected notices cover the configured renderer; if redistributing complete
upstream archives, preserve their additional per-file notices too.

Raylib includes separately licensed components: GLFW/rprand/rltexgpu use zlib-style
terms; stb uses MIT/public-domain alternatives; GLAD includes Khronos notices;
QOI and ImGui's embedded Proggy fonts use MIT terms; dirent has its own permissive
notice. The supplied notices and supplemental texts retain these terms.

GLFW's Windows fallback `dinput.h` and `xinput.h` headers are LGPL-2.1-or-later.
They contain declarations and small macros rather than a linked LGPL library;
[LGPL 2.1 section 5](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html)
addresses this use. Their complete sources and LGPL license are copied beside the
viewer. The testbed and bindings retain their own license.

When updating dependency pins or enabling additional modules, update the hashes,
collected notices, and supplemental licenses together.
