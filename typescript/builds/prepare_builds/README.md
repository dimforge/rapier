# Prepare builds

This project helps with making specific package: it takes a few parameter to create a (1) folder ready to compile.

It uses clap so you can pass `-h` to get more info about its parameters.

## usage

At workspace root: `cargo run -p prepare_builds -- -d dim2 -f simd`.

Or use provided scripts: `./builds/prepare_builds/prepare_all_projects.sh && ./builds/prepare_builds/build_all_projects.sh`

## Technical considerations

Askama/rinja was not chosen because compiled templates make it difficult to iterate on a folder and parse all templates.

This folder is in `builds/` only for the workspace member glob to not complain if `builds/` is inexistent or empty.
