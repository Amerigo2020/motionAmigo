//! Python bindings for motionAmigo.
use pyo3::prelude::*;

/// Native extension module, re-exported by the `motionamigo` Python package.
#[pymodule]
fn _motionamigo(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
