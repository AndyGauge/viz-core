# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

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
