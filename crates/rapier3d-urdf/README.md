## STL loader for the Rapier physics engine

Rapier is a set of 2D and 3D physics engines for games, animation, and robotics. The `rapier3d-urdf`
crate lets you convert an URDF file into a set of rigid-bodies, colliders, and joints, for usage with the
`rapier3d` physics engine.

## Optional cargo features

- `stl`: enables loading STL meshes referenced by the URDF file.

## Limitations

Are listed below some known limitations you might want to be aware of before picking this library. Contributions to
improve
these elements are very welcome!

- Mesh file types other than `stl` are not supported yet. Contributions are welcome. You my check the `rapier3d-stl`
  repository for an example of mesh loader.
- When inserting joints as multibody joints, they will be reset to their neutral position (all coordinates = 0).
- The following fields are currently ignored:
    - `Joint::dynamics`
    - `Joint::limit.effort` / `limit.velocity`
    - `Joint::mimic`
    - `Joint::safety_controller`

## Resources and discussions

- [Dimforge](https://dimforge.com): See all the open-source projects we are working on! Follow our announcements
  on our [blog](https://www.dimforge.com/blog).
- [User guide](https://www.rapier.rs/docs/): Learn to use Rapier in your project by reading the official User Guides.
- [Discord](https://discord.gg/vt9DJSW): Come chat with us, get help, suggest features, on Discord!
- [NPM packages](https://www.npmjs.com/search?q=%40dimforge): Check out our NPM packages for Rapier, if you need to
  use it with JavaScript/Typescript.

Please make sure to familiarize yourself with our [Code of Conduct](CODE_OF_CONDUCT.md)
and our [Contribution Guidelines](CONTRIBUTING.md) before contributing or participating in
discussions with the community.
