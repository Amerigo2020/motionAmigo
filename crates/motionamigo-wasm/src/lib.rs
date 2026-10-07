//! WebAssembly bindings for the motionAmigo browser demo.
use wasm_bindgen::prelude::*;

/// Returns the crate version.
#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}
