## 0.4.0

Renamed the crate from `rapier3d-stl` to `rapier3d-meshloader`, to better reflect its support for multiple formats.

### Added

- Add optional support for Collada and Wavefront files through new feature flags `collada` and `wavefront`.

### Modified

- Support for STL is now optional through feature `stl`.
- Features `stl`, `wavefront` and `collada` are enabled by default.

## 0.3.0

This is the initial release of the `rapier3d-stl` crate.

### Added

- Add `load_from_path` for creating a shape from a stl file.
- Add `load_from_reader` for creating a shape from an object implementing `Read`.
- Add `load_from_raw_mesh` for creating a shape from an already loaded `IndexedMesh`.
