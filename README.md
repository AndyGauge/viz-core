# viz-core

A small Rust/wasm-bindgen core for a browser map visualization: the point
cloud (GPS + temperature + vibration + timestamp readings) lives in WASM
linear memory after a single bulk copy from JS, and later interactions
(time slider, sensor selection, viewport pan/zoom) query it as scalars
instead of re-crossing the JS/WASM boundary with fresh data.
