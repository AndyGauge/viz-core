# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

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
