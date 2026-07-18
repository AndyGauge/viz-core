//! Intensity -> color, precomputed once at compile time. Painting a pixel
//! is then an array lookup, not a gradient function evaluated per pixel
//! per frame -- the whole point at 45,000 devices and a screen full of
//! cells, on the same frame budget this rewrite exists to protect.

const PALETTE_SIZE: usize = 256;

/// A display/quantization concern rather than a KDE-modeling one:
/// density.rs decides how heat spreads, this decides how heat gets
/// colored. Originally 3.0 (an unvalidated "three overlapping readings"
/// guess); measured against the real SensorDataGenerator output, a
/// single point's own intensity (density.rs's `intensity()`, max 1.0)
/// rarely exceeded ~0.3-0.5 even for a severe reading, at
/// VIBRATION_SATURATION_G's old value -- meaning a lone hot reading
/// almost never registered above the ramp's pale, near-transparent low
/// end, and only overlapping-and-summed readings at low zoom ever looked
/// visibly hot. 1.0 means a single severe reading (whose intensity now
/// approaches 1.0 after density.rs's matching recalibration) reaches full
/// saturation on its own; overlapping readings still sum past this and
/// saturate faster, which is the correct KDE behavior, not a regression.
const MAX_CELL_DENSITY: f64 = 1.0;

// Transparent blue -> cyan -> green -> yellow -> opaque red. Alpha rises
// with intensity so near-zero density is invisible (the map shows
// through) and the hottest cells are fully solid.
const STOPS: [(f64, u8, u8, u8, u8); 5] = [
    (0.00, 0, 0, 255, 0),
    (0.25, 0, 255, 255, 90),
    (0.50, 0, 255, 0, 150),
    (0.75, 255, 255, 0, 200),
    (1.00, 255, 0, 0, 255),
];

const PALETTE: [[u8; 4]; PALETTE_SIZE] = build_palette();

const fn build_palette() -> [[u8; 4]; PALETTE_SIZE] {
    let mut table = [[0u8; 4]; PALETTE_SIZE];
    let mut i = 0; // `while`, not `for` -- Iterator::next() isn't const-callable
    while i < PALETTE_SIZE {
        table[i] = ramp(i as f64 / (PALETTE_SIZE - 1) as f64);
        i += 1;
    }
    table
}

const fn clamp01(t: f64) -> f64 {
    if t < 0.0 {
        0.0
    } else if t > 1.0 {
        1.0
    } else {
        t
    }
}

const fn lerp_u8(a: u8, b: u8, f: f64) -> u8 {
    let value = a as f64 + (b as f64 - a as f64) * f;
    // Round-to-nearest via truncating cast rather than f64::round, whose
    // const-stability is newer/less certain than basic arithmetic; value
    // always lands in [0, 255] here so the +0.5 offset is always safe.
    (value + 0.5) as u8
}

const fn ramp(t: f64) -> [u8; 4] {
    let t = clamp01(t);
    let mut i = 0;
    while i < STOPS.len() - 1 {
        let (t0, r0, g0, b0, a0) = STOPS[i];
        let (t1, r1, g1, b1, a1) = STOPS[i + 1];
        if t >= t0 && t <= t1 {
            let f = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            return [lerp_u8(r0, r1, f), lerp_u8(g0, g1, f), lerp_u8(b0, b1, f), lerp_u8(a0, a1, f)];
        }
        i += 1;
    }
    let (_, r, g, b, a) = STOPS[STOPS.len() - 1];
    [r, g, b, a]
}

/// Quantizes a raw (possibly greater than 1.0, from overlapping kernels)
/// cell density into the precomputed palette.
pub fn rgba_for(density: f64) -> [u8; 4] {
    let t = clamp01(density / MAX_CELL_DENSITY);
    let idx = (t * (PALETTE_SIZE - 1) as f64) as usize;
    PALETTE[idx]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramp_endpoints_match_the_stops_table_exactly() {
        assert_eq!(ramp(0.0), [0, 0, 255, 0]);
        assert_eq!(ramp(1.0), [255, 0, 0, 255]);
    }

    #[test]
    fn ramp_hits_each_interior_stop_exactly() {
        assert_eq!(ramp(0.25), [0, 255, 255, 90]);
        assert_eq!(ramp(0.50), [0, 255, 0, 150]);
        assert_eq!(ramp(0.75), [255, 255, 0, 200]);
    }

    #[test]
    fn alpha_rises_monotonically_with_t() {
        let samples = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
        let alphas: Vec<u8> = samples.iter().map(|&t| ramp(t)[3]).collect();
        for w in alphas.windows(2) {
            assert!(w[1] >= w[0], "alpha should not decrease: {:?}", alphas);
        }
    }

    #[test]
    fn ramp_clamps_outside_zero_one() {
        assert_eq!(ramp(-1.0), ramp(0.0));
        assert_eq!(ramp(2.0), ramp(1.0));
    }

    #[test]
    fn rgba_for_matches_ramp_at_the_exact_endpoints() {
        // Only t=0.0 and t=1.0 are guaranteed to land on the identical
        // bucket rgba_for's quantization (truncating cast, 256 buckets)
        // would produce from ramp's continuous function directly -- 0*255
        // and 1*255 are both exact integers, but e.g. 0.5*255=127.5
        // truncates to a neighboring bucket, off by a channel or two from
        // ramp(0.5) itself. Don't assert exact equality at interior points.
        assert_eq!(rgba_for(0.0), ramp(0.0));
        assert_eq!(rgba_for(MAX_CELL_DENSITY), ramp(1.0));
    }

    #[test]
    fn palette_table_is_fully_populated() {
        // No leftover [0,0,0,0] gaps from an off-by-one in build_palette:
        // alpha should climb monotonically across the whole table, which
        // wouldn't hold if any stretch of entries were left as the zeroed
        // initial value instead of a real ramp() result.
        assert_eq!(PALETTE[0], [0, 0, 255, 0]);
        assert_eq!(PALETTE[PALETTE_SIZE - 1], [255, 0, 0, 255]);
        for w in PALETTE.windows(2) {
            assert!(w[1][3] >= w[0][3], "alpha should not decrease across the table");
        }
    }

    #[test]
    fn rgba_for_zero_density_is_fully_transparent_blue() {
        assert_eq!(rgba_for(0.0), [0, 0, 255, 0]);
    }

    #[test]
    fn rgba_for_clamps_at_and_above_max_cell_density() {
        assert_eq!(rgba_for(MAX_CELL_DENSITY), [255, 0, 0, 255]);
        assert_eq!(rgba_for(MAX_CELL_DENSITY * 10.0), [255, 0, 0, 255]);
    }

    #[test]
    fn rgba_for_clamps_below_zero() {
        assert_eq!(rgba_for(-5.0), rgba_for(0.0));
    }

    #[test]
    fn rgba_for_is_monotonically_more_opaque_with_more_density() {
        let a = rgba_for(0.5)[3];
        let b = rgba_for(1.5)[3];
        let c = rgba_for(2.5)[3];
        assert!(a <= b && b <= c);
    }
}
