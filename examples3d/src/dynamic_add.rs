use wgsparkl_testbed3d::{wgsparkl, Callbacks, RapierData};

use bevy::render::renderer::RenderDevice;
use nalgebra::vector;
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder};
use wgsparkl::models::DruckerPrager;
use wgsparkl::{
    models::ElasticCoefficients,
    pipeline::MpmData,
    solver::{Particle, ParticleDynamics, SimulationParams},
};
use wgsparkl_testbed3d::{AppState, PhysicsContext};

use crate::utils::default_scene;

pub fn dynamic_demo(
    device: RenderDevice,
    app_state: &mut AppState,
    _callbacks: &mut Callbacks,
) -> PhysicsContext {
    let mut rapier_data = RapierData::default();
    let device = device.wgpu_device();

    if !app_state.restarting {
        app_state.num_substeps = 20;
        app_state.gravity_factor = 1.0;
    };

    let params = SimulationParams {
        gravity: vector![0.0, -9.81, 0.0] * app_state.gravity_factor,
        dt: (1.0 / 60.0) / (app_state.num_substeps as f32),
    };

    default_scene::spawn_ground_and_walls(&mut rapier_data);

    callbacks.0.push(Box::new(
        move |_, physics: &mut PhysicsContext, _, _, queue| {
            physics.data.particles.push(
                queue,
                &Particle {
                    position,
                    dynamics: ParticleDynamics::with_density(radius, density),
                    model: ElasticCoefficients::from_young_modulus(2_000_000_000.0, 0.2),
                    plasticity: Some(DruckerPrager::new(2_000_000_000.0, 0.2)),
                    phase: None,
                    color: None,
                },
            )
        },
    ));

    let particles = vec![];
    let cell_width = 1.0;
    let data = MpmData::new(
        device,
        params,
        &particles,
        &rapier_data.bodies,
        &rapier_data.colliders,
        cell_width,
        60_000,
    );
    PhysicsContext {
        data,
        rapier_data,
        particles,
    }
}
