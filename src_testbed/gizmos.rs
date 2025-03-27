use bevy::prelude::*;

use crate::egui_text_gizmo::TextGizmos;

pub struct TestbedGizmos<'a> {
    text_gizmos: &'a mut TextGizmos,
}

impl<'a> TestbedGizmos<'a> {
    pub fn new(text_gizmos: &'a mut TextGizmos) -> Self {
        Self { text_gizmos }
    }

    pub fn add_text(&mut self, text: &str, world_pos: Vec3, size: f32, color: impl Into<Color>) {
        self.text_gizmos.add(text, world_pos, size, color);
    }
}
