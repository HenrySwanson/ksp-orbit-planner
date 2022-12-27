use kiss3d::renderer::LineRenderer;

use nalgebra::Point3;

pub fn path_iter_parametric<F, S>(
    f: F,
    t_start: S,
    t_end: S,
    num_segments: usize,
) -> impl Iterator<Item = Point3<f32>>
where
    F: Fn(S) -> Point3<f32>,
    S: nalgebra::RealField + simba::scalar::SupersetOf<usize> + Copy,
{
    assert!(
        num_segments >= 1,
        "Must have at least one segment, num_segments was {}",
        num_segments
    );
    let convert = nalgebra::convert::<usize, S>;
    (0..=num_segments)
        .map(move |i| convert(i) / convert(num_segments))
        // u ranges from 0 to 1 (inclusive)
        .map(move |u| t_start + u * (t_end - t_start))
        .map(f)
}

pub fn draw_path<I: Iterator<Item = Point3<f32>>>(
    line_renderer: &mut LineRenderer,
    points: I,
    color: &Point3<f32>,
) {
    let mut prev_pt = None;
    for pt in points {
        if let Some(prev_pt) = prev_pt {
            line_renderer.draw_line(prev_pt, pt, *color);
        }
        prev_pt = Some(pt);
    }
}
