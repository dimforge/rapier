# Rapier C bindings

C11 bindings for Rapier, with optional C++17 helpers, 2D/3D support, and single or
double precision.

You need Rust/Cargo, a C/C++ compiler, and CMake 3.25+. Run the commands below from
the repository root.

## Build the library

```sh
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --config Release --parallel
```

This builds the 3D, single-precision shared library, a small C example, and tests.
Both the Rust library and C/C++ code use release mode. CMake runs Cargo for you;
the library is under `build/c/cargo/release/`.

Add these options to the configuration command as needed:

| Option | Purpose |
| --- | --- |
| `-DRAPIER_DIMENSION=2` | Build 2D instead of 3D. |
| `-DRAPIER_PRECISION=64` | Use double precision instead of single precision. |
| `-DRAPIER_SHARED=OFF` | Link the static library. |
| `-DRAPIER_ENABLE_PARALLEL=ON` | Enable multithreaded physics; on by default for the testbed. |
| `-DRAPIER_SIMD_LANES=8` | Use eight SIMD lanes instead of four; requires single precision and excludes `enhanced-determinism`. |
| `-DRAPIER_FEATURES=fem,robotics` | Enable optional features; robotics requires 3D/single precision. |

Use separate build directories for different configurations. For debug builds,
set both `-DRAPIER_PROFILE=debug` and `-DCMAKE_BUILD_TYPE=Debug`, then build with
`--config Debug`.

## Install and use from CMake

```sh
cmake --install build/c --config Release --prefix /path/to/rapier-sdk
```

Configure your application with `-DCMAKE_PREFIX_PATH=/path/to/rapier-sdk`, then link:

```cmake
find_package(Rapier CONFIG REQUIRED)
target_link_libraries(your_app PRIVATE Rapier::rapier)
```

The target supplies the matching headers, compile definitions, and libraries.
Install each dimension/precision configuration to a separate prefix. When shipping
a shared-library build, include the library and configure its runtime search path
(on Windows, place the DLL beside your executable).

## Run the testbed

```sh
cmake -S c -B build/c3 -DRAPIER_BUILD_TESTBED=ON -DRAPIER_DIMENSION=3 \
  -DRAPIER_PROFILE=release -DCMAKE_BUILD_TYPE=Release
cmake --build build/c3 --target rapier_testbed --config Release --parallel
./build/c3/testbed/rapier_testbed
```

For 2D, use `build/c2` and `-DRAPIER_DIMENSION=2`. With a multi-configuration
generator such as Visual Studio, the executable is under `testbed/Release/`.

CMake downloads the graphics dependencies on the first configuration. macOS uses
system frameworks; Linux needs X11/OpenGL development packages.
See the [testbed guide](testbed/README.md) for controls, scene selection, and
headless runs, and [dependencies](testbed/dependencies.md) for offline builds.

## Documentation and examples

Generate the searchable API reference and usage guides:

```sh
cmake -S c/doxygen -B build/c-docs
cmake --build build/c-docs --target rapier_docs --parallel
```

Open `build/c-docs/html/index.html`. This requires Doxygen 1.9.4+, Python 3, and a
C compiler. It generates all four dimension/precision variants without building
the library or downloading testbed dependencies. Normal builds do not need Doxygen.

The reference covers API usage, ownership, errors, callbacks, and snapshots.
The CI workflow also provides a `rapier-c-api-docs` HTML artifact.

- [C example](examples/falling_ball.c)
- [C++ helpers](include/rapier.hpp)
- [C# interop example](examples/RapierNative.cs)

## Tests and header generation

```sh
ctest --test-dir build/c -C Release --output-on-failure
```

The headers are checked in; users do not need to generate them. When changing the
bindings or API comments in `c/src/`, regenerate the header with:

```sh
cargo install cbindgen --version 0.29.4 --locked
python3 c/tools/generate-header.py
```
