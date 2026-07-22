mod bbox;
mod density;
mod mercator;
mod palette;

use bbox::{contains_inclusive, make_bbox};
use density::{compute_grid, intensity, sigma_px, Grid, Splat};
use geo_types::Rect;
use js_sys::Float64Array;
use mercator::project;
use palette::rgba_for;
use wasm_bindgen::prelude::*;
use wasm_bindgen::{Clamped, JsCast};
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement, ImageData};

const ROW_LEN: usize = 6; // sensor_idx, lat, lng, temp_f, vibration_g, timestamp_ms

/// The full sensor point cloud, copied into WASM linear memory exactly once
/// at page load. Every later query (time slice, sensor selection, viewport)
/// runs as filters over this resident copy instead of re-crossing the
/// JS/WASM boundary with fresh data.
#[wasm_bindgen]
pub struct PointCloud {
    sensor_idx: Vec<u32>,
    lat: Vec<f64>,
    lng: Vec<f64>,
    temp_f: Vec<f64>,
    vibration_g: Vec<f64>,
    timestamp_ms: Vec<f64>,
    // Row indices grouped by sensor, in chronological order, so a single
    // instant in time can be located with a binary search per sensor
    // instead of a linear scan of the whole cloud.
    by_sensor: Vec<Vec<u32>>,
}

/// One interpolated reading for a single sensor at some instant, already
/// bbox-filtered. Shared by `snapshot_at` (flattens to its wire format) and
/// `density_grid` (projects + splats instead) so the by_sensor/bracket/
/// interpolate walk isn't duplicated a third time.
struct SensorReading {
    sensor_idx: u32,
    lat: f64,
    lng: f64,
    temp_f: f64,
    vibration_g: f64,
}

#[wasm_bindgen]
impl PointCloud {
    /// `flat` is a row-major Float64Array: one bulk copy out of JS memory
    /// (`Float64Array::to_vec`), not a per-property call per point.
    #[wasm_bindgen(constructor)]
    pub fn new(flat: &Float64Array, sensor_count: u32) -> PointCloud {
        let data = flat.to_vec();
        let rows = data.len() / ROW_LEN;

        let mut sensor_idx = Vec::with_capacity(rows);
        let mut lat = Vec::with_capacity(rows);
        let mut lng = Vec::with_capacity(rows);
        let mut temp_f = Vec::with_capacity(rows);
        let mut vibration_g = Vec::with_capacity(rows);
        let mut timestamp_ms = Vec::with_capacity(rows);

        for i in 0..rows {
            let base = i * ROW_LEN;
            sensor_idx.push(data[base] as u32);
            lat.push(data[base + 1]);
            lng.push(data[base + 2]);
            temp_f.push(data[base + 3]);
            vibration_g.push(data[base + 4]);
            timestamp_ms.push(data[base + 5]);
        }

        let mut by_sensor: Vec<Vec<u32>> = vec![Vec::new(); sensor_count as usize];
        for i in 0..rows {
            by_sensor[sensor_idx[i] as usize].push(i as u32);
        }

        PointCloud {
            sensor_idx,
            lat,
            lng,
            temp_f,
            vibration_g,
            timestamp_ms,
            by_sensor,
        }
    }

    pub fn len(&self) -> usize {
        self.lat.len()
    }

    /// Every raw reading matching a sensor bitmask, a timestamp range, and a
    /// viewport bounding box. Used by "show all readings" mode, scoped down
    /// to whatever the customer has actually selected and panned to.
    #[allow(clippy::too_many_arguments)]
    pub fn range(
        &self,
        sensor_mask: u32,
        start_ts: f64,
        end_ts: f64,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
    ) -> Float64Array {
        let bbox = make_bbox(min_lat, min_lng, max_lat, max_lng);
        let mut out: Vec<f64> = Vec::new();

        for i in 0..self.lat.len() {
            if sensor_mask & (1u32 << self.sensor_idx[i]) == 0 {
                continue;
            }
            let ts = self.timestamp_ms[i];
            if ts < start_ts || ts > end_ts {
                continue;
            }
            let (lat, lng) = (self.lat[i], self.lng[i]);
            if !contains_inclusive(&bbox, lat, lng) {
                continue;
            }
            out.push(self.sensor_idx[i] as f64);
            out.push(lat);
            out.push(lng);
            out.push(self.temp_f[i]);
            out.push(self.vibration_g[i]);
            out.push(ts);
        }

        Float64Array::from(out.as_slice())
    }

    /// One interpolated row per active sensor at instant `at_ts`, linearly
    /// blended between the two readings bracketing it. This is the time
    /// slider's "video playback" path: scrubbing asks for a single instant,
    /// not the raw once-a-minute cadence, so motion reads as continuous.
    ///
    /// Each returned row is `[sensor_idx, lat, lng, temp_f, vibration_g, at_ts]`
    /// (`ROW_LEN` = 6 floats), flattened into one `Float64Array`.
    ///
    /// # Examples
    ///
    /// Called from the frontend on every slider tick, with the map's current
    /// viewport and sensor selection as the filter:
    ///
    /// ```js
    /// const bounds = map.getBounds();
    /// const rows = cloud.snapshot_at(
    ///   atTs, sensorMask,
    ///   bounds.getSouth(), bounds.getWest(), bounds.getNorth(), bounds.getEast(),
    /// );
    /// for (let i = 0; i < rows.length; i += 6) {
    ///   const [sensorIdx, lat, lng, tempF, vibrationG] = rows.slice(i, i + 6);
    ///   // ...place a marker at [lat, lng]
    /// }
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn snapshot_at(
        &self,
        at_ts: f64,
        sensor_mask: u32,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
    ) -> Float64Array {
        let bbox = make_bbox(min_lat, min_lng, max_lat, max_lng);
        let readings = self.filtered_snapshot(at_ts, sensor_mask, &bbox);

        let mut out: Vec<f64> = Vec::with_capacity(readings.len() * ROW_LEN);
        for r in &readings {
            out.push(r.sensor_idx as f64);
            out.push(r.lat);
            out.push(r.lng);
            out.push(r.temp_f);
            out.push(r.vibration_g);
            out.push(at_ts);
        }

        Float64Array::from(out.as_slice())
    }

    /// A kernel-density heatmap grid over the viewport at instant `at_ts`:
    /// each active sensor's interpolated reading is projected into pixel
    /// space (Web Mercator, at the given `zoom`) and contributes a smooth
    /// Gaussian kernel to its neighboring cells, sized by its own
    /// `vibration_g` magnitude, blended with `temp_f` into one intensity
    /// value. This turns the filtered point set into density numbers on a
    /// projected plane -- it does not decide colors or paint any pixels;
    /// that's a later stage.
    ///
    /// The returned `Float64Array` is `[cols, rows, cell(0,0), cell(0,1),
    /// ..., cell(rows-1, cols-1)]`: a 2-value header giving the grid's
    /// dimensions (so the caller doesn't have to re-derive them from the
    /// viewport/zoom/cell size itself), followed by `cols * rows` row-major
    /// density values. Grid cells are `cell_size_px` pixels square.
    ///
    /// Known limitation: a reading just outside the requested viewport is
    /// dropped before projection, even though a high-vibration reading's
    /// kernel could otherwise bleed a little heat into the grid's visible
    /// edge cells. This matches `range`/`snapshot_at`'s existing hard cutoff
    /// at the viewport boundary today, so it's not a regression -- just
    /// known edge softness, fixable later by padding the *selection* bbox
    /// without changing this grid's dimensions.
    #[allow(clippy::too_many_arguments)]
    pub fn density_grid(
        &self,
        at_ts: f64,
        sensor_mask: u32,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
        zoom: f64,
        cell_size_px: f64,
    ) -> Float64Array {
        let bbox = make_bbox(min_lat, min_lng, max_lat, max_lng);
        let grid = self.compute_density_grid(at_ts, sensor_mask, &bbox, zoom, cell_size_px);

        let mut out: Vec<f64> = Vec::with_capacity(2 + grid.cells.len());
        out.push(grid.cols as f64);
        out.push(grid.rows as f64);
        out.extend(grid.cells);

        Float64Array::from(out.as_slice())
    }

    /// Paints the same kernel-density heatmap `density_grid` computes
    /// directly onto `canvas`, from Rust, in one call: project, KDE,
    /// palette, paint end to end. No JavaScript touches a pixel -- the
    /// pixel buffer is built as a `Clamped<&[u8]>` (mapping straight onto
    /// a `Uint8ClampedArray`, no separate JS glue), wrapped in an
    /// `ImageData`, and blitted with `put_image_data`.
    ///
    /// The canvas's own drawing buffer is resized (`set_width`/
    /// `set_height`) to the density grid's `cols`/`rows` -- one device
    /// pixel per KDE cell, not one per screen pixel. `cell_size_px` is
    /// already the resolution the Gaussian smoothing is visually
    /// meaningful at; painting at full viewport resolution would rerun
    /// KDE far more densely for a blur that's already smoother than that
    /// per cell. Stretch the small buffer to the viewport's actual size
    /// with the canvas element's CSS `width`/`height` and let the
    /// browser's own bilinear upscale do the rest.
    ///
    /// Pan, zoom, and scrubbing the time slider each call this same
    /// entry point with whatever changed (bbox, zoom, or `at_ts`/
    /// `sensor_mask`) -- one pipeline, three triggers. Deliberately not
    /// solved here: a pure pan with no time/sensor change recomputes
    /// density that hasn't actually changed. Caching the density grid
    /// and only reprojecting/repainting against it is a known, flagged
    /// optimization; this ships the correct version first.
    #[allow(clippy::too_many_arguments)]
    pub fn paint_heatmap(
        &self,
        canvas: &HtmlCanvasElement,
        at_ts: f64,
        sensor_mask: u32,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
        zoom: f64,
        cell_size_px: f64,
    ) -> Result<(), JsValue> {
        let bbox = make_bbox(min_lat, min_lng, max_lat, max_lng);
        let grid = self.compute_density_grid(at_ts, sensor_mask, &bbox, zoom, cell_size_px);

        let (cols, rows) = (grid.cols as u32, grid.rows as u32);
        let mut pixels = vec![0u8; grid.cells.len() * 4];
        for (i, &density) in grid.cells.iter().enumerate() {
            pixels[i * 4..i * 4 + 4].copy_from_slice(&rgba_for(density));
        }

        canvas.set_width(cols);
        canvas.set_height(rows);

        let ctx = canvas
            .get_context("2d")?
            .ok_or_else(|| JsValue::from_str("canvas 2d context unavailable"))?
            .dyn_into::<CanvasRenderingContext2d>()?;

        let image_data = ImageData::new_with_u8_clamped_array_and_sh(Clamped(&pixels), cols, rows)?;
        ctx.put_image_data(&image_data, 0.0, 0.0)
    }

    /// Draws each active sensor's raw readings as text at its projected
    /// position -- the numbers actually driving `paint_heatmap`'s color,
    /// not the color itself. A separate canvas from `paint_heatmap`'s on
    /// purpose: that canvas is deliberately sized to the KDE grid's own
    /// (coarse) resolution and CSS-stretched, which would make text blurry;
    /// this one is sized to the bbox's full projected pixel resolution so
    /// labels stay crisp.
    #[allow(clippy::too_many_arguments)]
    pub fn paint_heatmap_labels(
        &self,
        canvas: &HtmlCanvasElement,
        at_ts: f64,
        sensor_mask: u32,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
        zoom: f64,
        sensor_names: Vec<String>,
    ) -> Result<(), JsValue> {
        let bbox = make_bbox(min_lat, min_lng, max_lat, max_lng);
        let readings = self.filtered_snapshot(at_ts, sensor_mask, &bbox);

        let (min, max) = (bbox.min(), bbox.max());
        let (origin_x, origin_y) = project(min.x, max.y, zoom);
        let (max_x, _) = project(max.x, max.y, zoom);
        let (_, max_y) = project(min.x, min.y, zoom);
        let width_px = max_x - origin_x;
        let height_px = max_y - origin_y;

        // Resizing clears the canvas's existing content -- no separate
        // clear_rect needed.
        canvas.set_width(width_px.round().max(1.0) as u32);
        canvas.set_height(height_px.round().max(1.0) as u32);

        let ctx = canvas
            .get_context("2d")?
            .ok_or_else(|| JsValue::from_str("canvas 2d context unavailable"))?
            .dyn_into::<CanvasRenderingContext2d>()?;

        ctx.set_font("12px -apple-system, sans-serif");
        ctx.set_text_align("center");
        ctx.set_line_width(3.0);
        ctx.set_stroke_style_str("#000");
        ctx.set_fill_style_str("#fff");

        for r in &readings {
            let (x, y) = project(r.lng, r.lat, zoom);
            let (x, y) = (x - origin_x, y - origin_y);

            let name = sensor_names
                .get(r.sensor_idx as usize)
                .map(String::as_str)
                .unwrap_or("?");
            let readout = format!("{:.2}g / {:.0}\u{00b0}F", r.vibration_g, r.temp_f);

            // Stroke behind fill on each line, for legibility over any
            // map/heatmap color underneath.
            ctx.stroke_text(name, x, y - 14.0)?;
            ctx.fill_text(name, x, y - 14.0)?;
            ctx.stroke_text(&readout, x, y)?;
            ctx.fill_text(&readout, x, y)?;
        }

        Ok(())
    }

    /// Shared by `density_grid` and `paint_heatmap`: filters, projects, and
    /// splats -- everything short of flattening to a wire format (the
    /// former) or palette-mapping and painting (the latter).
    fn compute_density_grid(
        &self,
        at_ts: f64,
        sensor_mask: u32,
        bbox: &Rect<f64>,
        zoom: f64,
        cell_size_px: f64,
    ) -> Grid {
        let readings = self.filtered_snapshot(at_ts, sensor_mask, bbox);

        // Grid-local pixel origin: the viewport's top-left corner (west,
        // north) projected at the requested zoom. x depends only on lng,
        // y only on lat, so the two corners below suffice.
        let (min, max) = (bbox.min(), bbox.max());
        let (origin_x, origin_y) = project(min.x, max.y, zoom);
        let (max_x, _) = project(max.x, max.y, zoom);
        let (_, max_y) = project(min.x, min.y, zoom);
        let width_px = max_x - origin_x;
        let height_px = max_y - origin_y;

        let splats: Vec<Splat> = readings
            .iter()
            .map(|r| {
                let (x, y) = project(r.lng, r.lat, zoom);
                Splat {
                    x: x - origin_x,
                    y: y - origin_y,
                    intensity: intensity(r.temp_f, r.vibration_g),
                    sigma_px: sigma_px(r.vibration_g),
                }
            })
            .collect();

        compute_grid(&splats, width_px, height_px, cell_size_px)
    }

    /// One interpolated reading per active sensor at `at_ts`, bbox-filtered.
    /// Shared by `snapshot_at` and `compute_density_grid`.
    fn filtered_snapshot(&self, at_ts: f64, sensor_mask: u32, bbox: &Rect<f64>) -> Vec<SensorReading> {
        let mut out = Vec::new();

        for (sensor, rows) in self.by_sensor.iter().enumerate() {
            if rows.is_empty() || sensor_mask & (1u32 << sensor) == 0 {
                continue;
            }

            let (lo, hi) = self.bracket(rows, at_ts);
            let (lat, lng, temp_f, vibration_g) = self.interpolate(lo, hi, at_ts);

            if !contains_inclusive(bbox, lat, lng) {
                continue;
            }

            out.push(SensorReading {
                sensor_idx: sensor as u32,
                lat,
                lng,
                temp_f,
                vibration_g,
            });
        }

        out
    }

    fn bracket(&self, rows: &[u32], at_ts: f64) -> (u32, u32) {
        let split = rows.partition_point(|&row| self.timestamp_ms[row as usize] <= at_ts);
        let lo_pos = split.saturating_sub(1);
        let hi_pos = split.min(rows.len() - 1);
        (rows[lo_pos], rows[hi_pos])
    }

    fn interpolate(&self, lo: u32, hi: u32, at_ts: f64) -> (f64, f64, f64, f64) {
        let (lo, hi) = (lo as usize, hi as usize);
        let (t0, t1) = (self.timestamp_ms[lo], self.timestamp_ms[hi]);
        let f = if t1 > t0 {
            ((at_ts - t0) / (t1 - t0)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let lerp = |a: f64, b: f64| a + (b - a) * f;

        (
            lerp(self.lat[lo], self.lat[hi]),
            lerp(self.lng[lo], self.lng[hi]),
            lerp(self.temp_f[lo], self.temp_f[hi]),
            lerp(self.vibration_g[lo], self.vibration_g[hi]),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Two sensors, three readings each, one minute apart. Built directly
    // (bypassing `new`) since `new` takes a JS `Float64Array`, which needs a
    // JS runtime to construct -- `bracket`/`interpolate` are pure Rust and
    // don't need one.
    fn test_cloud() -> PointCloud {
        PointCloud {
            sensor_idx: vec![0, 0, 0, 1, 1, 1],
            lat: vec![10.0, 20.0, 30.0, 40.0, 40.0, 40.0],
            lng: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            temp_f: vec![60.0, 62.0, 64.0, 70.0, 70.0, 70.0],
            vibration_g: vec![0.1, 0.2, 0.3, 0.4, 0.4, 0.4],
            timestamp_ms: vec![0.0, 60_000.0, 120_000.0, 0.0, 60_000.0, 120_000.0],
            by_sensor: vec![vec![0, 1, 2], vec![3, 4, 5]],
        }
    }

    #[test]
    fn interpolates_halfway_between_two_readings() {
        let cloud = test_cloud();
        let (lo, hi) = cloud.bracket(&cloud.by_sensor[0], 30_000.0);
        let (lat, _lng, temp, _vib) = cloud.interpolate(lo, hi, 30_000.0);
        assert_eq!(lat, 15.0); // halfway from 10.0 to 20.0
        assert_eq!(temp, 61.0); // halfway from 60.0 to 62.0
    }

    #[test]
    fn clamps_to_the_first_reading_before_the_series_starts() {
        let cloud = test_cloud();
        let (lo, hi) = cloud.bracket(&cloud.by_sensor[0], -10_000.0);
        let (lat, ..) = cloud.interpolate(lo, hi, -10_000.0);
        assert_eq!(lat, 10.0);
    }

    #[test]
    fn clamps_to_the_last_reading_after_the_series_ends() {
        let cloud = test_cloud();
        let (lo, hi) = cloud.bracket(&cloud.by_sensor[0], 999_999.0);
        let (lat, ..) = cloud.interpolate(lo, hi, 999_999.0);
        assert_eq!(lat, 30.0);
    }

    #[test]
    fn snaps_exactly_onto_a_real_reading() {
        let cloud = test_cloud();
        let (lo, hi) = cloud.bracket(&cloud.by_sensor[1], 60_000.0);
        let (lat, ..) = cloud.interpolate(lo, hi, 60_000.0);
        assert_eq!(lat, 40.0);
    }

    #[test]
    fn filtered_snapshot_drops_sensors_outside_the_bbox() {
        let cloud = test_cloud();
        // sensor 0 sits at lat 10-30, sensor 1 at lat 40; restrict to a
        // bbox that only covers sensor 0's range.
        let bbox = make_bbox(0.0, -1.0, 35.0, 1.0);
        let readings = cloud.filtered_snapshot(60_000.0, 0b11, &bbox);
        assert_eq!(readings.len(), 1);
        assert_eq!(readings[0].sensor_idx, 0);
    }

    #[test]
    fn filtered_snapshot_respects_the_sensor_mask() {
        let cloud = test_cloud();
        let bbox = make_bbox(-90.0, -180.0, 90.0, 180.0);
        let readings = cloud.filtered_snapshot(60_000.0, 0b01, &bbox); // sensor 0 only
        assert_eq!(readings.len(), 1);
        assert_eq!(readings[0].sensor_idx, 0);
    }

    #[test]
    fn compute_density_grid_peaks_near_the_only_active_sensor() {
        let cloud = test_cloud();
        let bbox = make_bbox(35.0, -1.0, 45.0, 1.0); // covers sensor 1 (lat 40, lng 0) only
        let grid = cloud.compute_density_grid(60_000.0, 0b10, &bbox, 10.0, 4.0);

        assert_eq!(grid.cells.len(), grid.cols * grid.rows);
        // A 10-degree-tall bbox at zoom 10 should be many rows, not the
        // degenerate 1 that a sign error in height_px's north/south
        // subtraction would silently clamp down to (float->usize casts
        // saturate negative values to 0, then `.max(1)` masks it further --
        // see compute_grid's own comment on this exact footgun).
        assert!(grid.rows > 10, "rows = {}", grid.rows);

        // Sensor 1's exact reading at ts=60_000 (an exact snap, not an
        // interpolated blend -- see snaps_exactly_onto_a_real_reading).
        let expected_peak = intensity(70.0, 0.4);
        let actual_peak = grid.cells.iter().cloned().fold(0.0_f64, f64::max);

        assert!(actual_peak > 0.0);
        // A single point's own contribution never exceeds its intensity
        // (compute_grid's unnormalized-Gaussian invariant, tested directly
        // in density.rs).
        assert!(actual_peak <= expected_peak + 1e-9);
        // The nearest cell center should land close to the true peak, not
        // smeared away by an unlucky grid alignment.
        assert!(actual_peak > expected_peak * 0.9);
    }

    #[test]
    fn compute_density_grid_is_empty_when_no_sensor_is_selected() {
        let cloud = test_cloud();
        // A realistic, screen-scale bbox -- not the whole globe: at
        // zoom=10 a (-90,-180,90,180) bbox works out to a ~65536x65536
        // cell grid (~4.3 billion f64s), since compute_grid allocates
        // cols*rows regardless of splat count. No real caller passes a
        // global viewport at this zoom; keep the test's bbox matched to
        // what the frontend would actually request.
        let bbox = make_bbox(35.0, -1.0, 45.0, 1.0);
        let grid = cloud.compute_density_grid(60_000.0, 0, &bbox, 10.0, 4.0);
        assert!(grid.cells.iter().all(|&v| v == 0.0));
    }

    // Note: snapshot_at/range/density_grid/paint_heatmap themselves aren't
    // natively unit tested -- they construct a real js_sys::Float64Array
    // or take a real web_sys::HtmlCanvasElement, either of which needs a
    // JS/DOM runtime `cargo test` doesn't have. Their logic is covered
    // indirectly: filtered_snapshot and compute_density_grid (above) plus
    // mercator's, density's, and palette's own pure-Rust tests cover every
    // piece those methods assemble.
}
