# tsconfig json files

- tsconfig.common.json - shared TypeScript options
- tsconfig.pkg2d.json - config for compiling rapier2d-compat
- tsconfig.pkg3d.json - config for compiling rapier3d-compat
- tconfig.json - for IDE (VSCode) and unit tests. Includes Jest types.

## Generation steps

Check `./package.json scripts`, which are used by CI.

Summary:

- build rust wasm projects into their dedicated folder in `./builds/`
- copy common javascript and generate dimension specific javascript into a common `./pkg` folder
- copy that `pkg` folder in each folder from `./builds`
