use wgsparkl2d::rapier::prelude::{ColliderBuilder, RigidBodyBuilder};
use wgsparkl_testbed2d::{wgsparkl, Callbacks, RapierData};

use bevy::render::renderer::RenderDevice;
use nalgebra::{vector, Vector2};
use wgsparkl::models::DruckerPrager;
use wgsparkl::{
    models::ElasticCoefficients,
    pipeline::MpmData,
    solver::{Particle, SimulationParams},
};
use wgsparkl2d::solver::ParticleDynamics;
use wgsparkl_testbed2d::{AppState, PhysicsContext};

pub fn dynamic_demo(
    device: RenderDevice,
    app_state: &mut AppState,
    callbacks: &mut Callbacks,
) -> PhysicsContext {
    let mut rapier_data = RapierData::default();
    let device = device.wgpu_device();

    let offset_y = 46.0;
    let cell_width = 0.2;
    let mut particles = vec![];
    let position = vector![0.5, 0.5] * cell_width / 2.0 + Vector2::y() * offset_y;
    let density = 1000.0;
    let radius = cell_width / 4.0;
    particles.push(Particle {
        position,
        dynamics: ParticleDynamics::with_density(radius, density),
        model: ElasticCoefficients::from_young_modulus(10_000_000.0, 0.2),
        plasticity: Some(DruckerPrager::new(10_000_000.0, 0.2)),
        phase: None,
        color: None,
    });

    if !app_state.restarting {
        app_state.num_substeps = 10;
        app_state.gravity_factor = 1.0;
    };

    let params = SimulationParams {
        gravity: vector![0.0, -9.81] * app_state.gravity_factor,
        dt: (1.0 / 60.0) / (app_state.num_substeps as f32),
        padding: 0.0,
    };

    /*
     * Static platforms.
     */
    let rb = RigidBodyBuilder::fixed().translation(vector![35.0, -1.0]);
    let rb_handle = rapier_data.bodies.insert(rb);
    let co = ColliderBuilder::cuboid(42.0, 1.0);
    rapier_data
        .colliders
        .insert_with_parent(co, rb_handle, &mut rapier_data.bodies);

    let rb = RigidBodyBuilder::fixed()
        .translation(vector![-25.0, 45.0])
        .rotation(0.5);
    let rb_handle = rapier_data.bodies.insert(rb);
    let co = ColliderBuilder::cuboid(1.0, 52.0);
    rapier_data
        .colliders
        .insert_with_parent(co, rb_handle, &mut rapier_data.bodies);

    let rb = RigidBodyBuilder::fixed()
        .translation(vector![95.0, 45.0])
        .rotation(-0.5);
    let rb_handle = rapier_data.bodies.insert(rb);
    let co = ColliderBuilder::cuboid(1.0, 52.0);
    rapier_data
        .colliders
        .insert_with_parent(co, rb_handle, &mut rapier_data.bodies);

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

    let data = MpmData::new(
        device,
        params,
        &particles,
        1_000,
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
