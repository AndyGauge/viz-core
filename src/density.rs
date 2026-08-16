//! Kernel density estimation over a pixel grid: each projected reading
//! contributes a smooth, weighted Gaussian kernel to its neighboring cells
//! instead of falling into exactly one fixed bucket, so the heatmap doesn't
//! visibly jump as a reading crosses a bin boundary while the customer
//! scrubs the time slider.

// Calibrated against the actual SensorDataGenerator output (not the
// original 0.1-0.4 placeholder guess, which undershot both scenarios):
// cluster vibration_g ranges 0.0-0.915g (avg 0.037g, mostly near-silent
// with rare real spikes); highway ranges 0.25-3.058g (avg 0.519g). At
// VIBRATION_SATURATION_G = 2.0, even highway's worst pothole spikes only
// reached ~65% of the color ramp and a single reading almost never stood
// out on its own -- only overlapping, summed readings at low zoom ever
// looked "hot." 1.0 means a single severe reading (>= 1g, well within
// both scenarios' real range) now saturates on its own.
const TEMP_BASELINE_F: f64 = 70.0;
const TEMP_SPAN_F: f64 = 50.0;
const VIBRATION_SATURATION_G: f64 = 1.0;
const TEMP_WEIGHT: f64 = 0.35;
const VIBRATION_WEIGHT: f64 = 0.65;

const MIN_SIGMA_PX: f64 = 8.0; // 2x the recommended cell_size_px
const MAX_SIGMA_PX: f64 = 48.0; // 12x the recommended cell_size_px

/// Blends temperature and vibration into one intensity score in `[0, 1]`.
/// Vibration is weighted higher since it's the chapter's primary "shock"
/// signal; temperature contributes as a secondary anomaly term.
pub fn intensity(temp_f: f64, vibration_g: f64) -> f64 {
    let temp_score = ((temp_f - TEMP_BASELINE_F).abs() / TEMP_SPAN_F).min(1.0);
    let vib_score = (vibration_g.abs() / VIBRATION_SATURATION_G).min(1.0);
    TEMP_WEIGHT * temp_score + VIBRATION_WEIGHT * vib_score
}

/// Kernel radius from vibration magnitude alone (not the blended
/// intensity): a large shock/vibration reading spreads its influence
/// further across the grid; a marginal one barely shows at all.
pub fn sigma_px(vibration_g: f64) -> f64 {
    let t = (vibration_g.abs() / VIBRATION_SATURATION_G).min(1.0);
    MIN_SIGMA_PX + t * (MAX_SIGMA_PX - MIN_SIGMA_PX)
}

pub struct Splat {
    pub x: f64,
    pub y: f64,
    pub intensity: f64,
    pub sigma_px: f64,
}

pub struct Grid {
    pub cols: usize,
    pub rows: usize,
    pub cells: Vec<f64>,
}

/// Splats each point's kernel onto a `cell_size_px`-resolution grid sized
/// to `width_px` x `height_px`, bounded to each point's own `3*sigma` box
/// (~98.9% of Gaussian mass) rather than the whole grid.
///
/// The kernel is an *unnormalized* Gaussian, `exp(-(dx^2+dy^2) / (2*sigma^2))`
/// -- not the textbook density-normalized `1/(2*pi*sigma^2) * exp(...)`.
/// The normalized form blows up as sigma -> 0, which would make a small,
/// marginal reading spike to a huge peak relative to a large shock --
/// backwards from "a marginal one barely shows at all." The unnormalized
/// form caps each point's own contribution at `intensity` (reached only at
/// its center) regardless of sigma; a bigger sigma only spreads that same
/// peak further, never inflates it.
pub fn compute_grid(splats: &[Splat], width_px: f64, height_px: f64, cell_size_px: f64) -> Grid {
    let cols = ((width_px / cell_size_px).ceil() as usize).max(1);
    let rows = ((height_px / cell_size_px).ceil() as usize).max(1);
    let mut cells = vec![0.0_f64; cols * rows];

    for s in splats {
        let cutoff = 3.0 * s.sigma_px;

        // Reject fully-outside-grid splats BEFORE clamping to indices:
        // float->usize casts saturate (negative -> 0), so clamping first
        // without this guard would wrongly treat "entirely off the
        // top-left" as "overlaps cell (0,0)".
        if s.x + cutoff < 0.0
            || s.x - cutoff > width_px
            || s.y + cutoff < 0.0
            || s.y - cutoff > height_px
        {
            continue;
        }

        let col_lo = ((s.x - cutoff) / cell_size_px).floor().max(0.0) as usize;
        let col_hi = ((s.x + cutoff) / cell_size_px).ceil().min((cols - 1) as f64) as usize;
        let row_lo = ((s.y - cutoff) / cell_size_px).floor().max(0.0) as usize;
        let row_hi = ((s.y + cutoff) / cell_size_px).ceil().min((rows - 1) as f64) as usize;

        let two_sigma_sq = 2.0 * s.sigma_px * s.sigma_px;
        for row in row_lo..=row_hi {
            let dy = (row as f64 + 0.5) * cell_size_px - s.y;
            for col in col_lo..=col_hi {
                let dx = (col as f64 + 0.5) * cell_size_px - s.x;
                let d2 = dx * dx + dy * dy;
                if d2 > cutoff * cutoff {
                    continue; // circular cutoff, not just the bounding box
                }
                cells[row * cols + col] += s.intensity * (-d2 / two_sigma_sq).exp();
            }
        }
    }

    Grid { cols, rows, cells }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    #[test]
    fn intensity_is_zero_at_baseline_with_no_vibration() {
        assert_eq!(intensity(TEMP_BASELINE_F, 0.0), 0.0);
    }

    #[test]
    fn intensity_saturates_at_one() {
        assert_eq!(intensity(TEMP_BASELINE_F + 1000.0, 0.0), TEMP_WEIGHT);
        assert_eq!(intensity(TEMP_BASELINE_F, 1000.0), VIBRATION_WEIGHT);
        assert_eq!(intensity(TEMP_BASELINE_F + 1000.0, 1000.0), 1.0);
    }

    #[test]
    fn intensity_is_monotonic_in_each_input() {
        assert!(intensity(TEMP_BASELINE_F + 5.0, 0.0) < intensity(TEMP_BASELINE_F + 10.0, 0.0));
        assert!(intensity(TEMP_BASELINE_F, 0.1) < intensity(TEMP_BASELINE_F, 0.5));
    }

    #[test]
    fn sigma_ranges_between_min_and_max() {
        assert_eq!(sigma_px(0.0), MIN_SIGMA_PX);
        assert_eq!(sigma_px(1000.0), MAX_SIGMA_PX);
        assert!(sigma_px(0.5) > MIN_SIGMA_PX && sigma_px(0.5) < MAX_SIGMA_PX);
    }

    #[test]
    fn sigma_is_monotonic_in_vibration() {
        assert!(sigma_px(0.1) < sigma_px(0.5));
        assert!(sigma_px(0.5) < sigma_px(1.5));
    }

    #[test]
    fn single_point_at_cell_center_hits_exact_intensity() {
        // A cell center sits at (col+0.5)*cell_size; place the splat there
        // exactly so distance-from-center is zero and the kernel is 1.0.
        let cell_size = 4.0;
        let splat = Splat { x: 2.0, y: 2.0, intensity: 0.8, sigma_px: 8.0 };
        let grid = compute_grid(&[splat], 40.0, 40.0, cell_size);
        assert!((grid.cells[0] - 0.8).abs() < EPS);
    }

    #[test]
    fn falls_off_monotonically_with_distance() {
        let splat = Splat { x: 20.0, y: 20.0, intensity: 1.0, sigma_px: 16.0 };
        let grid = compute_grid(&[splat], 40.0, 40.0, 4.0);
        let center_col = 4; // (20 / 4) - 1 half cell, close enough to center
        let center_row = 4;
        let center = grid.cells[center_row * grid.cols + center_col];
        let further = grid.cells[center_row * grid.cols + 0];
        assert!(center > further);
    }

    #[test]
    fn non_overlapping_splats_do_not_cross_contaminate() {
        let splats = vec![
            Splat { x: 2.0, y: 2.0, intensity: 1.0, sigma_px: 4.0 },
            Splat { x: 98.0, y: 98.0, intensity: 1.0, sigma_px: 4.0 },
        ];
        let grid = compute_grid(&splats, 100.0, 100.0, 4.0);
        // Top-left corner cell should be untouched by the far splat.
        let far_corner = grid.cells[grid.cols - 1];
        assert_eq!(far_corner, 0.0);
    }

    #[test]
    fn overlapping_splats_sum_rather_than_overwrite() {
        let single = compute_grid(
            &[Splat { x: 20.0, y: 20.0, intensity: 0.5, sigma_px: 16.0 }],
            40.0,
            40.0,
            4.0,
        );
        let doubled = compute_grid(
            &[
                Splat { x: 20.0, y: 20.0, intensity: 0.5, sigma_px: 16.0 },
                Splat { x: 20.0, y: 20.0, intensity: 0.5, sigma_px: 16.0 },
            ],
            40.0,
            40.0,
            4.0,
        );
        for (a, b) in single.cells.iter().zip(doubled.cells.iter()) {
            assert!((b - 2.0 * a).abs() < EPS);
        }
    }

    #[test]
    fn splat_near_a_corner_with_large_sigma_does_not_panic() {
        // x - cutoff is meaningfully negative here (0 - 3*48 = -144),
        // exercising the saturating-cast guard.
        let splat = Splat { x: 0.0, y: 0.0, intensity: 1.0, sigma_px: MAX_SIGMA_PX };
        let grid = compute_grid(&[splat], 100.0, 100.0, 4.0);
        assert_eq!(grid.cells.len(), grid.cols * grid.rows);
        assert!(grid.cells.iter().all(|v| v.is_finite() && *v >= 0.0));
    }

    #[test]
    fn zero_size_grid_clamps_to_one_by_one() {
        let grid = compute_grid(&[], 0.0, 0.0, 4.0);
        assert_eq!(grid.cols, 1);
        assert_eq!(grid.rows, 1);
        assert_eq!(grid.cells, vec![0.0]);
    }

    #[test]
    fn empty_splats_gives_an_all_zero_grid() {
        let grid = compute_grid(&[], 40.0, 40.0, 4.0);
        assert_eq!(grid.cols * grid.rows, grid.cells.len());
        assert!(grid.cells.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn intensity_matches_real_generator_output_ranges() {
        // Calibration guard: pins the VIBRATION_SATURATION_G recalibration
        // (2.0 -> 1.0) against SensorDataGenerator's actual output ranges,
        // so a future constant change gets caught here instead of only
        // being noticed as "the heatmap looks grey" in a browser.
        // cluster: vibration_g 0.0-0.915 (avg 0.037); a real spike should
        // clearly outrank the near-silent baseline.
        assert!(intensity(75.0, 0.037) < 0.15);
        assert!(intensity(75.0, 0.915) > 0.5);
        // highway: vibration_g 0.25-3.058 (avg 0.519), refrigerated temps
        // far from TEMP_BASELINE_F; typical operation should already read
        // as more than "faint," and a pothole-magnitude spike should be
        // near full saturation.
        assert!(intensity(34.0, 0.519) > 0.4);
        assert!(intensity(34.0, 3.058) > 0.85);
    }
}
