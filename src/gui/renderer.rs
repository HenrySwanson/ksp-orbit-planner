use kiss3d::camera::Camera;
use kiss3d::renderer::Renderer;

pub struct CompoundRenderer {}

impl Renderer for CompoundRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {}
}
