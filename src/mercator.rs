//! Web Mercator projection: longitude/latitude to the flat pixel plane a
//! canvas actually draws on, and back. Standard slippy-map math, computed
//! once per filtered point inside Rust rather than by calling back into a
//! map library's projection object per point -- that per-point JS/WASM
//! round trip is exactly the tax this rewrite exists to remove.
//!
//! Pixel output is a plain `(f64, f64)` tuple, not `geo::Coord`: it's pixel
//! space, not geographic space, and reusing the same type for both (even
//! though the fields are structurally identical) invites mixing them up.

const TILE_SIZE_PX: f64 = 256.0;

pub fn world_size_px(zoom: f64) -> f64 {
    TILE_SIZE_PX * 2f64.powf(zoom)
}

/// Projects (lng, lat) to (x, y) pixel space at the given zoom level.
/// Latitude is clamped to Web Mercator's real domain (~+/-85.05113 deg,
/// via the standard `siny` clamp) rather than producing NaN/infinity near
/// the poles.
pub fn project(lng: f64, lat: f64, zoom: f64) -> (f64, f64) {
    let size = world_size_px(zoom);
    let x = (lng + 180.0) / 360.0 * size;

    let siny = lat.to_radians().sin().clamp(-0.9999, 0.9999);
    let y = (0.5 - ((1.0 + siny) / (1.0 - siny)).ln() / (4.0 * std::f64::consts::PI)) * size;

    (x, y)
}

/// Inverse of `project`. Not called by any production code path yet --
/// kept alongside `project` because their round trip is the strongest
/// available correctness test for both, needing no externally sourced
/// reference values. `#[allow(dead_code)]` because of exactly that: nothing
/// outside `#[cfg(test)]` calls it (yet).
#[allow(dead_code)]
pub fn unproject(x: f64, y: f64, zoom: f64) -> (f64, f64) {
    let size = world_size_px(zoom);
    let lng = x / size * 360.0 - 180.0;
    let lat = (std::f64::consts::PI * (1.0 - 2.0 * y / size))
        .sinh()
        .atan()
        .to_degrees();
    (lng, lat)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    #[test]
    fn zoom_doubling_doubles_both_axes() {
        for &(lng, lat) in &[(0.0, 0.0), (-73.9, 40.7), (139.7, 35.7), (-45.0, -22.9)] {
            for z in [0.0, 3.0, 8.0, 15.0] {
                let (x0, y0) = project(lng, lat, z);
                let (x1, y1) = project(lng, lat, z + 1.0);
                assert!((x1 - 2.0 * x0).abs() < EPS, "x at zoom {z} -> {}", z + 1.0);
                assert!((y1 - 2.0 * y0).abs() < EPS, "y at zoom {z} -> {}", z + 1.0);
            }
        }
    }

    #[test]
    fn equator_lands_on_the_vertical_midline() {
        for lng in [-180.0, -90.0, 0.0, 90.0, 179.9] {
            for z in [0.0, 5.0, 12.0] {
                let (_, y) = project(lng, 0.0, z);
                assert!((y - world_size_px(z) / 2.0).abs() < EPS);
            }
        }
    }

    #[test]
    fn antimeridian_lands_on_the_left_and_right_edges() {
        for z in [0.0, 5.0, 12.0] {
            let (x_west, _) = project(-180.0, 10.0, z);
            let (x_east, _) = project(180.0, 10.0, z);
            assert!((x_west - 0.0).abs() < EPS);
            assert!((x_east - world_size_px(z)).abs() < EPS);
        }
    }

    #[test]
    fn prime_meridian_lands_on_the_horizontal_midline() {
        for z in [0.0, 5.0, 12.0] {
            let (x, _) = project(0.0, 10.0, z);
            assert!((x - world_size_px(z) / 2.0).abs() < EPS);
        }
    }

    #[test]
    fn north_and_south_are_symmetric_about_the_equator() {
        for lat in [1.0, 22.9, 45.0, 60.0, 80.0] {
            for z in [0.0, 5.0, 12.0] {
                let (_, y_north) = project(0.0, lat, z);
                let (_, y_south) = project(0.0, -lat, z);
                assert!((y_north + y_south - world_size_px(z)).abs() < EPS);
            }
        }
    }

    #[test]
    fn further_north_is_higher_on_screen() {
        for z in [0.0, 5.0, 12.0] {
            let (_, y1) = project(0.0, 10.0, z);
            let (_, y2) = project(0.0, 20.0, z);
            assert!(y2 < y1);
        }
    }

    #[test]
    fn pole_clamp_stays_finite() {
        for z in [0.0, 5.0, 12.0] {
            let (x, y) = project(0.0, 90.0, z);
            assert!(x.is_finite() && y.is_finite());
            let (x, y) = project(0.0, -90.0, z);
            assert!(x.is_finite() && y.is_finite());
        }
    }

    #[test]
    fn project_and_unproject_round_trip() {
        for &(lng, lat) in &[
            (0.0, 0.0),
            (-73.9, 40.7),
            (139.7, 35.7),
            (-45.0, -22.9),
            (179.0, 84.0),
            (-179.0, -84.0),
        ] {
            for z in [0.0, 3.0, 8.0, 15.0] {
                let (x, y) = project(lng, lat, z);
                let (lng2, lat2) = unproject(x, y, z);
                assert!((lng - lng2).abs() < 1e-4, "lng at ({lng},{lat},{z})");
                assert!((lat - lat2).abs() < 1e-4, "lat at ({lng},{lat},{z})");
            }
        }
    }
}

