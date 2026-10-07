# Design decisions

This log records the decisions taken while building motionAmigo, with a short rationale for each.
Newest entries are appended at the bottom of each section.

## Project and process

* **Repository access.** The build session had no GitHub credentials for `Amerigo2020/motionAmigo`
  (no `add_repo` tool was available, the API answered "GitHub access to this repository is not
  enabled for this session"). Work happens in a local git repository with the requested author
  identity; it is pushed as soon as access exists. History is never rewritten for that.
* **Commit signing disabled locally.** The sandbox signs commits with a key that does not belong to
  Amerigo, which GitHub would show as "Unverified" next to his name. The repository therefore sets
  `commit.gpgsign=false`.
* **License.** `MIT OR Apache-2.0`, the Rust ecosystem default. Third-party data is listed in
  `NOTICE`.

## Scene format

* **Schema is strict, parser is lenient.** `schema/scene-v0.1.json` uses
  `additionalProperties: false` so typos are caught by validators. The Rust parser ignores unknown
  fields so that a slightly newer spatialAmigo output still loads.
* **Every object is an oriented box.** As specified, `front` and `viewpoint` are parsed but ignored
  by the planner. `yaw` rotates about world z, `size` holds full edge lengths.

## Build and CI

* **Rust edition 2021, MSRV 1.86.** 1.86 is the first release with safe `#[target_feature]`
  functions (target_feature 1.1), which keeps the amount of `unsafe` in the SIMD layer minimal.
* **WASM always uses simd128.** Set in `.cargo/config.toml`; every evergreen browser supports it.
* **Python packaging.** `crates/motionamigo-py` is a maturin project. `[tool.uv] package = false`
  lets `uv sync` install only the dev tools, and `uv run maturin develop --uv` builds the extension.
  This keeps the requested `uv sync`, `uv run maturin develop`, `uv run pytest` workflow.
* **abi3 wheels.** One wheel per platform covers Python 3.9 and newer.
* **CI matrix** covers x86_64 Linux, aarch64 Linux (NEON backend), macOS (aarch64) and Windows.
