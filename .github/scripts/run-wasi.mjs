// Runs a `wasm32-wasip1` binary under Node's WASI, forwarding argv, env and stdout.
//
// Used as `CARGO_TARGET_WASM32_WASIP1_RUNNER` so `cargo test --target wasm32-wasip1` can
// execute the test binary it builds. Node is used rather than a standalone runtime because
// it is already on every GitHub runner; any WASI preview1 runtime (wasmtime, wasmer) works
// just as well if you prefer one locally.
//
// Requires Node >= 22, where `node:wasi` is importable without a command-line flag.
import { WASI } from 'node:wasi';
import { readFile } from 'node:fs/promises';
import { argv, env } from 'node:process';

const [, , wasmPath, ...args] = argv;
const wasi = new WASI({
  version: 'preview1',
  // argv[0] is conventional and unused by libtest; the rest are the harness's own flags.
  args: ['test', ...args],
  env,
  returnOnExit: true,
});

const wasm = await WebAssembly.compile(await readFile(wasmPath));
const instance = await WebAssembly.instantiate(wasm, wasi.getImportObject());
process.exitCode = wasi.start(instance);
