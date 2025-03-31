//! This work is dual licensed under the Apache License v2.0 and the MIT license (SPDX: Apache-2.0, MIT).
//!
//! Original source: https://gist.github.com/jakkos-net/7f1d2806fae0288a11f3eb0840a11b04
//!
//! Changes:
//! - plugin addition.
//!
//! usage example:
//! fn example_system(mut text_gizmos: ResMut<TextGizmos>) {
//!     text_gizmos.add("this is a text gizmo!", Vec3::ZERO, 10.0, Color::WHITE);
//! }
//!
//! Remember to include [`TextGizmosPlugin`]:

pub struct TextGizmosPlugin;

impl Plugin for TextGizmosPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.init_resource::<TextGizmos>()
            .add_systems(Last, draw_text_gizmos);
    }
}

use bevy::{prelude::*, window::PrimaryWindow};
use bevy_egui::{egui, EguiContext};
use std::collections::VecDeque;

/// Displays text in immediate mode.
#[derive(Resource, Default)]
pub struct TextGizmos(VecDeque<TextGizmo>);

impl TextGizmos {
    pub fn add(&mut self, text: &str, pos: Vec3, size: f32, color: impl Into<Color>) {
        self.0.push_back(TextGizmo {
            text: text.to_string(),
            pos,
            size,
            color: color.into(),
        });
    }
}

pub struct TextGizmo {
    text: String,
    pos: Vec3,
    size: f32,
    color: Color,
}

#[derive(Component)]
pub struct TextGizmoCam;

pub fn draw_text_gizmos(
    window: Single<(&Window, &mut EguiContext), With<PrimaryWindow>>,
    cam: Single<(&Camera, &GlobalTransform), With<TextGizmoCam>>,
    mut text_gizmos: ResMut<TextGizmos>,
) {
    let (window, mut egui_ctx) = window.into_inner();
    let (cam, cam_tf) = cam.into_inner();
    let res = window.resolution.size();
    let painter = egui_ctx.get_mut().layer_painter(egui::LayerId {
        order: egui::Order::Background,
        id: egui::Id::new("text gizmos"),
    });

    for text_gizmo in text_gizmos.0.drain(..) {
        let world_dist = (text_gizmo.pos - cam_tf.translation()).length();
        let font_size = text_gizmo.size * 100.0 / world_dist; // treat font size as at 100 units away
        if font_size < 5.0 {
            continue; // if text too small, don't draw
        }
        let Some(ndc) = cam.world_to_ndc(cam_tf, text_gizmo.pos) else {
            continue;
        };
        if ndc.x.abs() > 1. || ndc.y.abs() > 1. || ndc.z < 0. {
            continue; // if center of text is off screen, don't draw
        }
        let screen_pos = egui::pos2(((ndc.x + 1.) / 2.) * res.x, ((-ndc.y + 1.) / 2.) * res.y);
        painter.text(
            screen_pos,
            egui::Align2::CENTER_CENTER,
            text_gizmo.text,
            egui::FontId::monospace(font_size),
            bevy_color_to_egui(text_gizmo.color),
        );
    }
    text_gizmos.0.clear();
}

/// This is not an accurate color conversion, but i don't need it to be
fn bevy_color_to_egui(color: impl Into<Color>) -> egui::Color32 {
    let color: Color = color.into();
    let color = color.to_srgba();
    fn f32_to_u8(f: f32) -> u8 {
        let f = f.clamp(0.0, 1.0);
        let u = (f * 256.0).round() as u8;
        u
    }
    egui::Color32::from_rgba_unmultiplied(
        f32_to_u8(color.red),
        f32_to_u8(color.green),
        f32_to_u8(color.blue),
        f32_to_u8(color.alpha),
    )
}
