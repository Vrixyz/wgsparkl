use bevy::color::palettes;
use wgsparkl3d::solver::ParticlePhase;
use wgsparkl3d::wgrapier::dynamics::body::{BodyCoupling, BodyCouplingEntry};
use wgsparkl_testbed3d::gizmos::TestbedGizmos;
use wgsparkl_testbed3d::{wgsparkl, Callback, Callbacks, RapierData, Rendering};

use bevy::render::renderer::RenderDevice;
use nalgebra::{vector, Vector2, Vector3};
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder};
use wgsparkl::models::DruckerPrager;
use wgsparkl::{
    models::ElasticCoefficients,
    pipeline::MpmData,
    solver::{Particle, ParticleDynamics, SimulationParams},
};
use wgsparkl_testbed3d::{AppState, PhysicsContext};

use crate::utils::default_scene;

#[derive(Debug)]
pub struct ParticlesConfiguration {
    pub coords: Vector2<i32>,
    pub density: f32,
    pub model: ElasticCoefficients,
    pub plasticity: Option<DruckerPrager>,
    pub phase: Option<ParticlePhase>,
    pub description: String,
}

pub fn configurations_demo(
    device: RenderDevice,
    app_state: &mut AppState,
    callbacks: &mut Callbacks,
) -> PhysicsContext {
    let mut rapier_data = RapierData::default();
    let device = device.wgpu_device();

    let grid_size_x = 10;
    let grid_size_y = 10;
    let grid_size_z = 10;
    let num_particles = grid_size_x * grid_size_y * grid_size_z;

    let particle_positions = (0..num_particles)
        .map(|i| {
            let x = i % grid_size_x;
            let y = (i / grid_size_x) % grid_size_y;
            let z = (i / (grid_size_x * grid_size_y)) % grid_size_z;
            Vector3::<f32>::new(x as f32, y as f32 + 1f32, z as f32)
        })
        .collect::<Vec<_>>();

    if !app_state.restarting {
        app_state.num_substeps = 16;
        app_state.gravity_factor = 1.0;
    };

    let cell_width = 0.5f32;
    let mut particles = vec![];
    let mut configurations = vec![];
    let get_position_for_line = |z: f32| -> bevy::math::Vec3 {
        bevy::math::Vec3::new(
            -2f32 * grid_size_x as f32 * 3f32 * 2f32,
            5f32,
            z * grid_size_z as f32 * 0.7f32 * 3f32 * 2f32 + grid_size_z as f32 / 2f32,
        )
    };
    let mut display_text_at_world_pos = |world_pos: bevy::math::Vec3, text: String| {
        callbacks.0.push(Callback {
            callback: Box::new(
                move |render: Option<Rendering>, _physics, _timestamps, _app_state| {
                    render.unwrap().text_gizmos.add_text(
                        &text,
                        world_pos,
                        42f32,
                        palettes::css::CORAL,
                    );
                },
            ),
            run_when_paused: true,
        });
    };
    {
        let young_modulus = 1_000_000_000.0;
        let z = -1f32;
        display_text_at_world_pos(
            get_position_for_line(z),
            format!(
                "With plasticity\nmodulus = {}M",
                young_modulus / 1_000_000.0
            ),
        );
        // line with plasticity, varying poisson
        for x in -1..2 {
            let poisson_ratio = match x {
                -1 => 0.0,
                0 => 0.2,
                1 => 0.4,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            let plasticity = Some(DruckerPrager {
                h0: 35.0f32.to_radians(),
                h1: 9.0f32.to_radians(),
                h2: 0.2,
                h3: 10.0f32.to_radians(),
                ..DruckerPrager::new(model.lambda, model.mu)
            });
            configurations.push(ParticlesConfiguration {
                coords: Vector2::<i32>::new(x, z as i32),
                density: 3700f32,
                model,
                plasticity,
                phase: None,
                description: format!("poisson: {}", poisson_ratio),
            });
        }
    }

    {
        let poisson_ratio = 0f32;
        let z = 0f32;
        display_text_at_world_pos(
            get_position_for_line(z),
            format!("With plasticity\npoisson = {}", poisson_ratio),
        );
        // line with plasticity, varying young modulus
        for x in -1..2 {
            let young_modulus = match x {
                -1 => 1_000_000.0,
                0 => 10_000_000.0,
                1 => 100_000_000.0,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            let plasticity = Some(DruckerPrager {
                h0: 35.0f32.to_radians(),
                h1: 9.0f32.to_radians(),
                h2: 0.2,
                h3: 10.0f32.to_radians(),
                ..DruckerPrager::new(model.lambda, model.mu)
            });
            configurations.push(ParticlesConfiguration {
                coords: Vector2::<i32>::new(x, z as i32),
                density: 3700f32,
                model,
                plasticity,
                phase: None,
                description: format!("modulus: {}M", young_modulus / 1_000_000f32),
            });
        }
    }

    {
        let poisson_ratio = 0f32;
        let z = 1f32;
        display_text_at_world_pos(
            get_position_for_line(z),
            format!("Without plasticity.\npoisson = {}", poisson_ratio),
        );

        // line without plasticity, varying young modulus
        for x in -1..2 {
            let young_modulus = match x {
                -1 => 1_000_000.0,
                0 => 50_000_000.0,
                1 => 200_000_000.0,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            configurations.push(ParticlesConfiguration {
                coords: Vector2::<i32>::new(x, z as i32),
                density: 3700f32,
                model,
                plasticity: None,
                phase: Some(ParticlePhase {
                    phase: 1.0,
                    max_stretch: f32::MAX,
                }),
                description: format!("modulus: {}M", young_modulus / 1_000_000.0),
            });
        }
    }
    {
        let young_modulus = 1_000_000.0;
        let z = 2f32;
        display_text_at_world_pos(
            get_position_for_line(z),
            format!(
                "Without plasticity.\nmodulus = {}M",
                young_modulus / 1_000_000.0
            ),
        );
        // line without plasticity, varying poisson_ratio
        for x in -1..2 {
            let poisson_ratio = match x {
                -1 => -0.2f32,
                0 => 0.3,
                1 => 0.48,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            configurations.push(ParticlesConfiguration {
                coords: Vector2::<i32>::new(x, z as i32),
                density: 3700f32,
                model,
                plasticity: None,
                phase: Some(ParticlePhase {
                    phase: 1.0,
                    max_stretch: f32::MAX,
                }),
                description: format!("poisson: {}", poisson_ratio),
            });
        }
    }

    for c in configurations.iter() {
        let x = c.coords.x as f32 * 3f32;
        let z = c.coords.y as f32 * 3f32;
        let offset = vector![
            x * grid_size_x as f32,
            3f32,
            z * grid_size_z as f32 * 0.7f32
        ] * 2f32;
        for particle in &particle_positions {
            let position = vector![particle.x, particle.y, particle.z];
            let density = c.density;
            particles.push(Particle {
                position: nalgebra::Rotation::from_axis_angle(
                    &Vector3::z_axis(),
                    1f32.to_radians(),
                ) * position
                    + offset,
                dynamics: ParticleDynamics::with_density(1.0, density),
                model: c.model,
                plasticity: c.plasticity,
                phase: c.phase,
                color: None,
            });
            display_text_at_world_pos(
                bevy::math::Vec3::new(offset.x, 5f32, offset.z + 10f32 + grid_size_z as f32),
                c.description.clone(),
            );
        }
    }

    if !app_state.restarting {
        app_state.num_substeps = 20;
        app_state.gravity_factor = 1.0;
    };

    let params = SimulationParams {
        gravity: vector![0.0, -9.81, 0.0] * app_state.gravity_factor,
        dt: (1.0 / 60.0) / (app_state.num_substeps as f32),
    };

    let scale = 3f32;
    rapier_data.insert_body_and_collider(
        RigidBodyBuilder::fixed().translation(vector![0.0, -4.0, 0.0] * scale),
        ColliderBuilder::cuboid(100.0 * scale, 4.0 * scale, 100.0 * scale),
    );

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
