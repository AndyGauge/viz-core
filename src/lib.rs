use js_sys::Float64Array;
use wasm_bindgen::prelude::*;

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
            if lat < min_lat || lat > max_lat || lng < min_lng || lng > max_lng {
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
        let mut out: Vec<f64> = Vec::new();

        for (sensor, rows) in self.by_sensor.iter().enumerate() {
            if rows.is_empty() || sensor_mask & (1u32 << sensor) == 0 {
                continue;
            }

            let (lo, hi) = self.bracket(rows, at_ts);
            let (lat, lng, temp, vib) = self.interpolate(lo, hi, at_ts);

            if lat < min_lat || lat > max_lat || lng < min_lng || lng > max_lng {
                continue;
            }

            out.push(sensor as f64);
            out.push(lat);
            out.push(lng);
            out.push(temp);
            out.push(vib);
            out.push(at_ts);
        }

        Float64Array::from(out.as_slice())
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
