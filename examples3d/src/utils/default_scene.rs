#![allow(unused)]

use bevy::prelude::*;
use nalgebra::{vector, Vector3};
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder};
use wgsparkl3d::{
    models::ElasticCoefficients,
    solver::{Particle, ParticleDynamics, ParticlePhase},
};
use wgsparkl_testbed3d::{AppState, RapierData};

pub const SAMPLE_PER_UNIT: f32 = 10.0;

/// Spawns a ground and 4 walls.
pub fn spawn_ground_and_walls(rapier_data: &mut RapierData, scale: f32) {
    rapier_data.insert_body_and_collider(
        RigidBodyBuilder::fixed().translation(vector![0.0, -4.0, 0.0] * scale),
        ColliderBuilder::cuboid(100.0 * scale, 4.0 * scale, 100.0 * scale),
    );

    rapier_data.insert_body_and_collider(
        RigidBodyBuilder::fixed().translation(vector![0.0, 5.0, -35.0] * scale),
        ColliderBuilder::cuboid(35.0 * scale, 5.0 * scale, 0.5 * scale),
    );
    rapier_data.insert_body_and_collider(
        RigidBodyBuilder::fixed().translation(vector![0.0, 5.0, 35.0] * scale),
        ColliderBuilder::cuboid(35.0 * scale, 5.0 * scale, 0.5 * scale),
    );
    rapier_data.insert_body_and_collider(
        RigidBodyBuilder::fixed().translation(vector![-35.0, 5.0, 0.0] * scale),
        ColliderBuilder::cuboid(0.5 * scale, 5.0 * scale, 35.0 * scale),
    );
    rapier_data.insert_body_and_collider(
        RigidBodyBuilder::fixed().translation(vector![35.0, 5.0, 0.0] * scale),
        ColliderBuilder::cuboid(0.5 * scale, 5.0 * scale, 35.0 * scale),
    );
}

pub fn create_particle(pos: &Vec3, color: Option<Color>, radius: f32) -> Particle {
    let density = 3700.0;
    Particle {
        position: pos.to_array().into(),
        dynamics: ParticleDynamics::with_density(radius, density),
        model: ElasticCoefficients::from_young_modulus(10_000_000.0, 0.22),
        plasticity: None,
        phase: Some(ParticlePhase {
            phase: 1f32,
            max_stretch: f32::MAX,
        }),
        color: color.map(|c| c.to_linear().to_u8_array()),
    }
}
