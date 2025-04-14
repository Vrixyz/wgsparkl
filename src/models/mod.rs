use crate::solver::{Particle, ParticlePhase};
pub use drucker_prager::{DruckerPrager, DruckerPragerPlasticState, WgDruckerPrager};
pub use linear_elasticity::WgLinearElasticity;
pub use neo_hookean_elasticity::WgNeoHookeanElasticity;
use wgcore::tensor::GpuVector;
use wgpu::{BufferUsages, Device};

mod drucker_prager;
mod linear_elasticity;
mod neo_hookean_elasticity;

pub struct GpuModels {
    pub linear_elasticity: GpuVector<ElasticCoefficients>,
    pub drucker_prager_plasticity: GpuVector<DruckerPrager>,
    pub drucker_prager_plastic_state: GpuVector<DruckerPragerPlasticState>,
    pub phases: GpuVector<ParticlePhase>,
}

impl GpuModels {
    pub fn from_particles(device: &Device, particles: &[Particle], maximum_size: usize) -> Self {
        let additional_unused_particles = maximum_size - particles.len();
        let models: Vec<_> = particles
            .iter()
            .map(|p| p.model)
            .chain(vec![
                ElasticCoefficients::from_young_modulus(
                    2_000_000_000.0,
                    0.2
                );
                additional_unused_particles
            ])
            .collect();
        let plasticity: Vec<_> = particles
            .iter()
            .map(|p| p.plasticity.unwrap_or(DruckerPrager::new(-1.0, -1.0)))
            .chain(vec![
                DruckerPrager::new(-1.0, -1.0);
                additional_unused_particles
            ])
            .collect();
        let plastic_states: Vec<_> = particles
            .iter()
            .map(|_| DruckerPragerPlasticState::default())
            .chain(vec![
                DruckerPragerPlasticState::default();
                additional_unused_particles
            ])
            .collect();
        let phases: Vec<_> = particles
            .iter()
            .map(|p| {
                p.phase.unwrap_or(ParticlePhase {
                    phase: 0.0,
                    max_stretch: -1.0,
                })
            })
            .chain(vec![
                ParticlePhase {
                    phase: 0.0,
                    max_stretch: -1.0,
                };
                additional_unused_particles
            ])
            .collect();
        Self {
            linear_elasticity: GpuVector::init(
                device,
                &models,
                BufferUsages::STORAGE | BufferUsages::COPY_DST,
            ),
            drucker_prager_plasticity: GpuVector::init(
                device,
                &plasticity,
                BufferUsages::STORAGE | BufferUsages::COPY_DST,
            ),
            drucker_prager_plastic_state: GpuVector::init(
                device,
                &plastic_states,
                BufferUsages::STORAGE | BufferUsages::COPY_DST,
            ),
            phases: GpuVector::init(
                device,
                &phases,
                BufferUsages::STORAGE | BufferUsages::COPY_DST,
            ),
        }
    }

    pub fn push(&mut self, queue: &wgpu::Queue, particle: &Particle, current_size: u32) {
        let model = particle.model;
        let plasticity = particle
            .plasticity
            .unwrap_or(DruckerPrager::new(-1.0, -1.0));
        let plastic_state = DruckerPragerPlasticState::default();
        let phase = particle.phase.unwrap_or(ParticlePhase {
            phase: 0.0,
            max_stretch: -1.0,
        });

        // Push model
        let offset = current_size as u64 * size_of::<ElasticCoefficients>() as u64;
        let bytes = bytemuck::bytes_of(&model);
        queue.write_buffer(&self.linear_elasticity.buffer(), offset, bytes);
        // Push plasticity
        let offset = current_size as u64 * size_of::<DruckerPrager>() as u64;
        let bytes = bytemuck::bytes_of(&plasticity);
        queue.write_buffer(&self.drucker_prager_plasticity.buffer(), offset, bytes);
        // Push plastic_state
        let offset = current_size as u64 * size_of::<DruckerPragerPlasticState>() as u64;
        let bytes = bytemuck::bytes_of(&plastic_state);
        queue.write_buffer(&self.drucker_prager_plastic_state.buffer(), offset, bytes);
        // Push phase
        let offset = current_size as u64 * size_of::<ParticlePhase>() as u64;
        let bytes = bytemuck::bytes_of(&phase);
        queue.write_buffer(&self.phases.buffer(), offset, bytes);
    }
}

fn lame_lambda_mu(young_modulus: f32, poisson_ratio: f32) -> (f32, f32) {
    (
        young_modulus * poisson_ratio / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio)),
        shear_modulus(young_modulus, poisson_ratio),
    )
}

fn shear_modulus(young_modulus: f32, poisson_ratio: f32) -> f32 {
    young_modulus / (2.0 * (1.0 + poisson_ratio))
}

#[derive(bytemuck::Pod, bytemuck::Zeroable, Copy, Clone, PartialEq, Debug)]
#[repr(C)]
pub struct ElasticCoefficients {
    pub lambda: f32,
    pub mu: f32,
}

impl ElasticCoefficients {
    pub fn from_young_modulus(young_modulus: f32, poisson_ratio: f32) -> Self {
        let (lambda, mu) = lame_lambda_mu(young_modulus, poisson_ratio);
        Self { lambda, mu }
    }
}
