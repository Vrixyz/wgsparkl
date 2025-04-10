use core::f32;

use wgsparkl_testbed3d::{wgsparkl, Callbacks, RapierData};

use bevy::render::renderer::RenderDevice;
use nalgebra::{vector, Isometry, Point3, Rotation3, UnitQuaternion, Vector3};
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder};
use wgsparkl::models::DruckerPrager;
use wgsparkl::{
    models::ElasticCoefficients,
    pipeline::MpmData,
    solver::{Particle, ParticleDynamics, SimulationParams},
};
use wgsparkl_testbed3d::{AppState, PhysicsContext};

pub fn bowl_demo(
    device: RenderDevice,
    app_state: &mut AppState,
    _callbacks: &mut Callbacks,
) -> PhysicsContext {
    let mut rapier_data = RapierData::default();
    let device = device.wgpu_device();

    let nxz = 45;
    let cell_width = 1.0;
    let mut particles = vec![];
    let young_modulus = 2_000_000_000.0;
    let poisson_ratio = 0.1;
    let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
    let plasticity = Some(DruckerPrager::new(young_modulus, poisson_ratio));

    let density = 2700.0;
    let radius = cell_width / 4.0;
    let dynamics = ParticleDynamics::with_density(radius, density);
    for i in 0..nxz {
        for j in 0..50 {
            for k in 0..nxz {
                let position = vector![
                    i as f32 + 0.5 - nxz as f32 / 2.0,
                    j as f32 + 0.5 + 10.0,
                    k as f32 + 0.5 - nxz as f32 / 2.0
                ] * cell_width
                    / 2.0;
                particles.push(Particle {
                    position,
                    dynamics,
                    model,
                    plasticity,
                    phase: None,
                    color: None,
                });
            }
        }
    }

    if !app_state.restarting {
        app_state.num_substeps = 10;
        app_state.gravity_factor = 1.0;
    };

    let params = SimulationParams {
        gravity: vector![0.0, -9.81, 0.0] * app_state.gravity_factor,
        dt: (1.0 / 60.0) / (app_state.num_substeps as f32),
    };

    // Bowl
    let radial_segments = 30;
    let vertical_segments = 15;
    let radius = 1.0;
    let height = 0.5;
    let scale = 30f32;
    let (vtx, idx) = generate_bowl_mesh(radial_segments, vertical_segments, radius, height);
    let vtx = vtx.into_iter().map(|p| p * scale).collect::<Vec<_>>();
    let rb = RigidBodyBuilder::fixed();
    let rb_handle = rapier_data.bodies.insert(rb);
    let co = ColliderBuilder::trimesh(vtx, idx).unwrap();
    rapier_data
        .colliders
        .insert_with_parent(co, rb_handle, &mut rapier_data.bodies);

    // Leaves to stir
    let rb = RigidBodyBuilder::kinematic_velocity_based()
        .translation(vector![0.0, 8.0, 0.0])
        .angvel(vector![0.0, -1.0, 0.0]);
    let rb_handle = rapier_data.bodies.insert(rb);
    // leaf 1
    let (vtx, idx) = generate_leaf_mesh();
    let vtx = vtx
        .into_iter()
        .map(|p| p * scale * 0.55)
        .collect::<Vec<_>>();
    let co = ColliderBuilder::trimesh(vtx, idx)
        .unwrap()
        .position(Isometry::from_parts(
            vector![scale * 0.2, 0.0, 0.0].into(),
            UnitQuaternion::from_scaled_axis(Vector3::new(-45f32.to_radians(), 0.0, 0.0)),
        ));
    rapier_data
        .colliders
        .insert_with_parent(co, rb_handle, &mut rapier_data.bodies);
    // leaf 2
    let (vtx, idx) = generate_leaf_mesh();
    let vtx = vtx
        .into_iter()
        .map(|p| p * scale * 0.55)
        .collect::<Vec<_>>();
    let co = ColliderBuilder::trimesh(vtx, idx)
        .unwrap()
        .position(Isometry::from_parts(
            vector![-scale * 0.2, 0.0, 0.0].into(),
            Rotation3::from_scaled_axis(Vector3::new(0f32, 180f32.to_radians(), 0.0).into())
                * UnitQuaternion::from_scaled_axis(Vector3::new(-45f32.to_radians(), 0.0, 0.0)),
        ));
    rapier_data
        .colliders
        .insert_with_parent(co, rb_handle, &mut rapier_data.bodies);

    rapier_data.insert_body_and_collider(
        RigidBodyBuilder::fixed().translation(vector![0.0, -4.0, 0.0]),
        ColliderBuilder::cuboid(100.0, 4.0, 100.0),
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

fn generate_bowl_mesh(
    radial_segments: usize,
    vertical_segments: usize,
    radius: f32,
    height: f32,
) -> (Vec<Point3<f32>>, Vec<[u32; 3]>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Generate vertices
    for j in 0..=vertical_segments {
        let v = j as f32 / vertical_segments as f32;
        let current_radius = radius * (1.0 - v);
        let y = height * (current_radius * current_radius * current_radius * current_radius);

        for i in 0..=radial_segments {
            let theta = (i as f32 / radial_segments as f32) * std::f32::consts::TAU;
            let x = current_radius * theta.cos();
            let z = current_radius * theta.sin();
            vertices.push(Point3::new(x, y, z));
        }
    }

    // Generate indices (triangles)
    for j in 0..vertical_segments {
        for i in 0..radial_segments {
            let current = j * (radial_segments + 1) + i;
            let next = current + radial_segments + 1;

            // First triangle
            indices.push([current as u32, next as u32, (current + 1) as u32]);

            // Second triangle
            indices.push([(current + 1) as u32, next as u32, (next + 1) as u32]);
        }
    }

    (vertices, indices)
}

fn generate_leaf_mesh() -> (Vec<Point3<f32>>, Vec<[u32; 3]>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let resolution = 64; // Number of points along the leaf

    // See https://en.wikipedia.org/wiki/Superformula
    let a = 1f32;
    let b = 1f32;
    let m = 1.0f32;
    let n1 = 0.5f32;
    let n2 = 0.5f32;
    let n3 = 0.5f32;

    let mut leaf_vertices = Vec::new();

    leaf_vertices.push(Point3::new(0.0, 0.0, 0.0));
    for i in 0..resolution {
        let phi = (i as f32 / resolution as f32) * std::f32::consts::TAU;

        let xp = (((m * phi) / 4f32).cos() / a).abs().powf(n2);
        let yp = (((m * phi) / 4f32).sin() / b).abs().powf(n3);

        let rphi = (xp + yp).powf(-1.0 / n1);

        let x = rphi * phi.cos();
        let z = rphi * phi.sin();

        leaf_vertices.push(Point3::new(x, 0.0, z));
    }

    indices.append(
        (1..leaf_vertices.len() - 1)
            .map(|i| [0, (i + 1) as u32, i as u32])
            .collect::<Vec<_>>()
            .as_mut(),
    );
    indices.push([0, 1, (leaf_vertices.len() - 1) as u32]);

    vertices.append(&mut leaf_vertices);
    (vertices, indices)
}
