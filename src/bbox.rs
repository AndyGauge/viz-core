use geo_types::{Coord, Rect};

/// `geo::Coord`/`Point` are `(x, y)` = `(lng, lat)` -- opposite order from
/// every `(lat, lng)` field on `PointCloud`. `Rect::new` accepts corners in
/// either order and normalizes them, which is the only part of `geo::Rect`
/// this crate actually needs.
pub fn make_bbox(min_lat: f64, min_lng: f64, max_lat: f64, max_lng: f64) -> Rect<f64> {
    Rect::new(
        Coord { x: min_lng, y: min_lat },
        Coord { x: max_lng, y: max_lat },
    )
}

/// Inclusive of the boundary -- deliberately NOT `geo::Contains`, whose
/// `Rect` impl uses strict `<`/`>` and would silently drop a point sitting
/// exactly on a viewport edge (an entirely ordinary case from
/// `map.getBounds()`).
pub fn contains_inclusive(bbox: &Rect<f64>, lat: f64, lng: f64) -> bool {
    let (min, max) = (bbox.min(), bbox.max());
    lng >= min.x && lng <= max.x && lat >= min.y && lat <= max.y
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::Contains;

    fn test_bbox() -> Rect<f64> {
        make_bbox(10.0, 20.0, 30.0, 40.0) // min_lat, min_lng, max_lat, max_lng
    }

    #[test]
    fn contains_the_center() {
        assert!(contains_inclusive(&test_bbox(), 20.0, 30.0));
    }

    #[test]
    fn contains_all_four_edges() {
        let bbox = test_bbox();
        assert!(contains_inclusive(&bbox, 10.0, 30.0)); // south edge
        assert!(contains_inclusive(&bbox, 30.0, 30.0)); // north edge
        assert!(contains_inclusive(&bbox, 20.0, 20.0)); // west edge
        assert!(contains_inclusive(&bbox, 20.0, 40.0)); // east edge
    }

    #[test]
    fn rejects_just_outside_each_edge() {
        let bbox = test_bbox();
        assert!(!contains_inclusive(&bbox, 9.999, 30.0));
        assert!(!contains_inclusive(&bbox, 30.001, 30.0));
        assert!(!contains_inclusive(&bbox, 20.0, 19.999));
        assert!(!contains_inclusive(&bbox, 20.0, 40.001));
    }

    #[test]
    fn raw_geo_contains_is_strict_and_excludes_the_boundary() {
        // Pins down *why* contains_inclusive exists instead of `.contains()`:
        // the same boundary point contains_inclusive accepts, geo::Contains
        // rejects.
        let bbox = test_bbox();
        let edge = Coord { x: 30.0, y: 10.0 }; // (lng, lat) on the south edge
        assert!(contains_inclusive(&bbox, edge.y, edge.x));
        assert!(!bbox.contains(&edge));
    }
}
