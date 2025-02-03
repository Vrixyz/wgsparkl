use crate::collision::WgCollide;
use crate::dim_shader_defs;
use crate::grid::grid::{GpuGrid, WgGrid};
use crate::grid::kernel::WgKernel;
use crate::models::{GpuModels, WgDruckerPrager, WgLinearElasticity, WgNeoHookeanElasticity};
use crate::solver::params::{GpuSimulationParams, WgParams};
use crate::solver::GpuParticles;
use crate::solver::WgParticle;
use encase::ShaderType;
use rapier::math::{AngVector, Vector};
use wgcore::kernel::{KernelInvocationBuilder, KernelInvocationQueue};
use wgcore::tensor::GpuVector;
use wgcore::Shader;
use wgparry::substitute_aliases;
use wgpu::{BufferUsages, ComputePipeline, Device};
use wgrapier::dynamics::{GpuBodySet, WgBody};

#[derive(Shader)]
#[shader(
    derive(WgBody),
    src = "rigid_impulses.wgsl",
    src_fn = "substitute_aliases",
    shader_defs = "dim_shader_defs"
)]
pub struct WgRigidImpulses {
    pub update: ComputePipeline,
    pub reset: ComputePipeline,
}

#[derive(ShaderType, Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct RigidImpulse {
    linear: Vector<f32>,
    angular: AngVector<f32>,
}

pub struct GpuImpulses {
    pub incremental_impulses: GpuVector<RigidImpulse>,
    pub total_impulses: GpuVector<RigidImpulse>,
    pub total_impulses_staging: GpuVector<RigidImpulse>,
}

impl GpuImpulses {
    pub fn new(device: &wgpu::Device) -> Self {
        const MAX_BODY_COUNT: usize = 16; // CPIC doesnt support more.
        let impulses = [RigidImpulse::default(); MAX_BODY_COUNT];
        Self {
            incremental_impulses: GpuVector::encase(device, &impulses, BufferUsages::STORAGE),
            total_impulses: GpuVector::encase(
                device,
                &impulses,
                BufferUsages::STORAGE | BufferUsages::COPY_SRC,
            ),
            total_impulses_staging: GpuVector::encase(
                device,
                &impulses,
                BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            ),
        }
    }
}

impl WgRigidImpulses {
    pub fn queue_update<'a>(
        &'a self,
        queue: &mut KernelInvocationQueue<'a>,
        impulses: &GpuImpulses,
        bodies: &GpuBodySet,
    ) {
        KernelInvocationBuilder::new(queue, &self.update)
            .bind0([
                impulses.total_impulses.buffer(),
                impulses.incremental_impulses.buffer(),
                bodies.vels().buffer(),
                bodies.mprops().buffer(),
            ])
            .queue(1);
    }

    pub fn queue_reset<'a>(
        &'a self,
        queue: &mut KernelInvocationQueue<'a>,
        impulses: &GpuImpulses,
    ) {
        KernelInvocationBuilder::new(queue, &self.reset)
            .bind0([impulses.total_impulses.buffer()])
            .queue(1);
    }
}

wgcore::test_shader_compilation!(WgRigidImpulses, wgcore, crate::dim_shader_defs());
