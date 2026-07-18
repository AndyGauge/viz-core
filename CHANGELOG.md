# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.3.0] - 2026-07-18

The density grid becomes a picture: intensity mapped to color via a
precomputed lookup table, painted directly onto a `<canvas>` from Rust —
no JavaScript touches a pixel.

### Added

- `PointCloud::paint_heatmap` — runs project, KDE, palette, and paint end
  to end in one call, given a canvas element plus the same filter
  parameters `snapshot_at`/`density_grid` take. Builds the pixel buffer as
  `wasm_bindgen::Clamped<&[u8]>` (mapping straight onto a
  `Uint8ClampedArray`), wraps it in a `web_sys::ImageData`
  (`new_with_u8_clamped_array_and_sh`), and blits it with
  `put_image_data`. The canvas's drawing buffer is sized to the density
  grid's own resolution (one device pixel per KDE cell, not one per
  screen pixel) — CSS stretches it to the viewport.
- `palette` module — a precomputed, compile-time (`const fn`) 256-entry
  RGBA lookup table: transparent blue → cyan → green → yellow → opaque
  red, alpha rising with intensity. Painting a pixel is an array lookup,
  not a gradient function evaluated per pixel per frame.
- `compute_density_grid` — the shared filter/project/splat core
  `density_grid` and `paint_heatmap` both call, extracted the same way
  `filtered_snapshot` is already shared between `snapshot_at` and
  `density_grid`. Returns a pure `density::Grid`, making it natively
  testable for the first time (unlike `range`/`snapshot_at`/
  `density_grid`/`paint_heatmap` themselves, which construct real
  `Float64Array`/`ImageData`/canvas objects and need a JS+DOM runtime
  `cargo test` doesn't have).
- `PointCloud::paint_heatmap_labels` — draws each active sensor's raw
  `vibration_g`/`temp_f` readings as text at its projected position, the
  actual numbers driving `paint_heatmap`'s color rather than the color
  itself. A separate canvas on purpose: `paint_heatmap`'s is deliberately
  sized to the coarse KDE grid resolution and CSS-stretched, which would
  blur text, so this one is sized to the bbox's full projected pixel
  resolution instead.
- Frontend: a third "Heatmap" view mode on the WASM engine, alongside
  "Show all readings" and "Time playback." Pan, zoom, and scrubbing the
  time slider each call `paint_heatmap`/`paint_heatmap_labels` with
  whatever changed — one pipeline, three triggers.
- 11 more unit tests (41 total, up from 30): the palette ramp/
  quantization math, plus an end-to-end `compute_density_grid` pipeline
  test asserting a single active sensor's peak cell value and bound.

### Notes

- `web-sys` added as a dependency (`CanvasRenderingContext2d`,
  `HtmlCanvasElement`, `ImageData` features only). Unlike `geo` last
  release, this is genuinely small: DOM bindings are extern declarations,
  not bundled algorithm code, so the shipped `public/wasm/viz_core_bg.wasm`
  only grows from ~43KB to ~48KB.
- `paint_heatmap_labels` grows the shipped wasm further, to ~77KB — this
  is the crate's first use of float-to-string formatting (`format!` for
  the on-canvas readouts), and `core::fmt`'s float formatting is known to
  cost real code size on `wasm32-unknown-unknown` (no hardware-assisted
  path). Not a dependency; inherent to formatting floats at all.
- Deliberately not solved here: a pure pan with no time/sensor change
  recomputes density that hasn't actually changed. Caching the grid and
  only reprojecting/repainting against it is a known, flagged
  optimization — this release ships the correct version first.
- A plain, absolutely-positioned `<canvas>` doesn't move with Leaflet's
  own panes during an active drag; the frontend fades it out on
  `movestart`/`zoomstart` and repaints on `moveend`/`zoomend` rather than
  tracking the pan in real time. A custom Leaflet layer to fix that is
  out of scope for this release.

## [0.2.0] - 2026-07-18

Turns a filtered point set into density numbers on a projected pixel
grid. Not a picture yet — color mapping and canvas painting are a later
release — but the two stages before it: projecting surviving readings
into screen space, then estimating density across a grid.

### Added

- `PointCloud::density_grid` — a kernel-density heatmap grid over the
  viewport at a single instant (mirrors `snapshot_at`'s shape, not
  `range`'s). Each active sensor's interpolated reading is projected into
  pixel space and contributes an unnormalized Gaussian kernel to its
  neighboring cells, radius scaled from its own `vibration_g` magnitude,
  blended with `temp_f` into one intensity value. Kernel density
  estimation instead of fixed-grid binning, so the heatmap doesn't
  visibly jump as a reading crosses a bin boundary while the customer
  scrubs the time slider.
- `mercator` module — Web Mercator projection (`project`/`unproject`/
  `world_size_px`), computed once per filtered point inside Rust rather
  than by calling back into the map library's projection object per
  point. Takes zoom as an input, since meters-per-pixel changes with
  every zoom level.
- `bbox` module — `geo::Rect`/`Coord` in place of the hand-rolled bbox
  tuple/comparison arithmetic `range()` and `snapshot_at()` each had
  duplicated. Deliberately not `geo`'s own `Contains` trait, whose `Rect`
  impl is strict and would silently drop a point sitting exactly on a
  viewport edge — an ordinary case from `map.getBounds()`.
- 26 more unit tests (30 total, up from 4): the projection math via
  zoom-doubling/equator/antimeridian/pole-clamp/round-trip invariants
  rather than external reference numbers; the bbox's inclusive-vs-strict
  boundary semantics, pinned against the real upstream `Contains` trait;
  density's intensity/sigma/splat accumulation, including a regression
  test for a saturating-cast footgun at grid corners.

### Notes

- Blends `temp_f` + `vibration_g` only — no separate `shock_g` field
  exists in the data model.
- `geo` (not `geo-types`) is a dev-only dependency, used solely by the
  one test above; its default features pull in triangulation/spatial-
  index/parallelism libraries this crate has no use for. `geo-types`
  itself is nearly free — the wasm size cost of this release is from
  `wasm32-unknown-unknown` needing a software implementation of the
  `sin`/`ln`/`sinh`/`atan`/`exp`/`powf` calls the projection and KDE code
  make, not from either dependency. The shipped, wasm-bindgen-processed
  artifact grows from ~27KB to ~43KB.

## [0.1.0] - 2026-07-18

Initial release: a wasm-bindgen core for a browser map visualization. The
full sensor point cloud (GPS + temperature + vibration + timestamp
readings) is copied into WASM linear memory exactly once; later queries
run as filters over that resident copy instead of re-crossing the
JS/WASM boundary with fresh data.

### Added

- `PointCloud::new` — takes the point cloud as a single row-major
  `Float64Array` (`[sensor_idx, lat, lng, temp_f, vibration_g,
  timestamp_ms]` per row) and bulk-copies it into WASM linear memory in
  one call, rather than one JS/WASM call per property per point.
- `PointCloud::range` — every raw reading matching a sensor bitmask, a
  timestamp range, and a viewport bounding box. Backs "show all
  readings" mode.
- `PointCloud::snapshot_at` — one interpolated row per active sensor at
  an arbitrary instant, linearly blended between the two readings
  bracketing it, so a time slider reads as continuous motion instead of
  jumping once a minute. Bracket lookup is a per-sensor binary search
  (`partition_point`) against a `by_sensor` index built at construction,
  not a linear scan of the whole cloud.
- Unit tests covering the interpolation/bracketing math: halfway
  blending, clamping to the first/last reading outside the series, and
  snapping exactly onto a real reading. Enabled by `crate-type =
  ["cdylib", "rlib"]`, so `cargo test` runs on the native target
  alongside the `wasm32-unknown-unknown` release build.
- Doc example on `snapshot_at` showing the JS call site (viewport bbox
  + sensor mask in, a flat row buffer out).
