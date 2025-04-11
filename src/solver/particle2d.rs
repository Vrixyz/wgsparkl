use crate::dim_shader_defs;
use crate::models::{DruckerPrager, ElasticCoefficients};
use crate::solver::ParticlePhase;
use encase::ShaderType;
use nalgebra::{vector, Matrix2, Point2, Vector2};
use rapier::geometry::{ColliderSet, Polyline, Segment};
use wgcore::tensor::{GpuScalar, GpuVector};
use wgcore::Shader;
use wgparry::shape::ShapeBuffers;
use wgpu::{BufferUsages, Device};
use wgrapier::dynamics::body::BodyCouplingEntry;
use wgrapier::dynamics::GpuBodySet;

#[derive(Copy, Clone, PartialEq, Debug, ShaderType)]
#[repr(C)]
pub struct ParticleDynamics {
    pub velocity: Vector2<f32>,
    pub def_grad: Matrix2<f32>,
    pub affine: Matrix2<f32>,
    pub cdf: Cdf,
    pub init_volume: f32,
    pub init_radius: f32,
    pub mass: f32,
}

impl ParticleDynamics {
    pub fn with_density(radius: f32, density: f32) -> Self {
        let exponent = if cfg!(feature = "dim2") { 2 } else { 3 };
        let init_volume = (radius * 2.0).powi(exponent); // NOTE: the particles are square-ish.
        Self {
            velocity: Vector2::zeros(),
            def_grad: Matrix2::identity(),
            affine: Matrix2::zeros(),
            init_volume,
            init_radius: radius,
            mass: init_volume * density,
            cdf: Cdf::default(),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Default, ShaderType)]
#[repr(C)]
pub struct Cdf {
    pub normal: Vector2<f32>,
    pub rigid_vel: Vector2<f32>,
    pub signed_distance: f32,
    pub affinity: u32,
}

#[derive(Copy, Clone, Debug)]
pub struct Particle {
    pub position: Vector2<f32>,
    pub dynamics: ParticleDynamics,
    pub model: ElasticCoefficients,
    pub plasticity: Option<DruckerPrager>,
    pub phase: Option<ParticlePhase>,
    pub color: Option<[u8; 4]>,
}

pub struct GpuRigidParticles {
    pub local_sample_points: GpuVector<Point2<f32>>,
    pub sample_points: GpuVector<Point2<f32>>,
    pub rigid_particle_needs_block: GpuVector<u32>,
    pub node_linked_lists: GpuVector<u32>,
    pub sample_ids: GpuVector<GpuSampleIds>,
}

impl GpuRigidParticles {
    pub fn new(device: &Device) -> Self {
        Self::from_rapier(
            device,
            &ColliderSet::default(),
            &GpuBodySet::new(device, &[], &[], &ShapeBuffers::default()),
            &[],
            1.0,
        )
    }

    pub fn from_rapier(
        device: &Device,
        colliders: &ColliderSet,
        gpu_bodies: &GpuBodySet,
        coupling: &[BodyCouplingEntry],
        sampling_step: f32,
    ) -> Self {
        let mut sampling_buffers = SamplingBuffers::default();

        for (collider_id, (coupling, gpu_data)) in coupling
            .iter()
            .zip(gpu_bodies.shapes_data().iter())
            .enumerate()
        {
            let collider = &colliders[coupling.collider];
            if let Some(polyline) = collider.shape().as_polyline() {
                let rngs = gpu_data.polyline_rngs();
                let sampling_params = SamplingParams {
                    collider_id: collider_id as u32,
                    base_vid: rngs[0],
                    sampling_step,
                };
                sample_polyline(polyline, &sampling_params, &mut sampling_buffers)
            }
        }

        Self {
            local_sample_points: GpuVector::encase(
                device,
                &sampling_buffers.samples,
                BufferUsages::STORAGE,
            ),
            sample_points: GpuVector::init(
                device,
                &sampling_buffers.samples,
                BufferUsages::STORAGE,
            ),
            node_linked_lists: GpuVector::uninit(
                device,
                sampling_buffers.samples.len() as u32,
                BufferUsages::STORAGE,
            ),
            sample_ids: GpuVector::encase(
                device,
                &sampling_buffers.samples_ids,
                BufferUsages::STORAGE,
            ),
            // NOTE: this is a packed bitmask so each u32 contains
            //       the flag for 32 particles.
            rigid_particle_needs_block: GpuVector::uninit(
                device,
                sampling_buffers.samples.len().div_ceil(32) as u32,
                BufferUsages::STORAGE,
            ),
        }
    }

    pub fn len(&self) -> u64 {
        self.sample_points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

pub struct GpuParticles {
    pub positions: GpuVector<Vector2<f32>>,
    pub dynamics: GpuVector<ParticleDynamics>,
    pub sorted_ids: GpuVector<u32>,
    pub node_linked_lists: GpuVector<u32>,
    /// The number of particles in use.
    ///
    /// As GPU buffers have to be re-created from the ground up to be resized,
    /// it's an expensive operation, so we can allocate big buffers and keep track
    /// of the used size.
    pub current_size: GpuScalar<u32>,
    /// As current_size is a uniform buffer, we can only change it from CPU, so it's easy to cache.
    ///
    /// If we don't cache it, we should retrieve it from current_size which is stored on GPU.
    pub current_size_cached: u32,
    /// Current actual size of the buffer on the GPU.
    pub maximum_size: u32,
}

impl GpuParticles {
    pub fn is_empty(&self) -> bool {
        self.current_size_cached == 0
    }

    pub fn len(&self) -> usize {
        self.current_size_cached as usize
    }

    /// Creates a new GPU particle buffer.
    ///
    /// `maximum_size` should be equal or greater than `particles.len()`.
    /// Setting a greater value will allow to add more particles later.
    pub fn from_particles(device: &Device, particles: &[Particle], maximum_size: usize) -> Self {
        let particles_len = particles.len();
        assert!(
            maximum_size >= particles_len,
            "`maximum_size` should be equal or greater than `particles.len()`."
        );
        let additional_unused_particles = maximum_size - particles_len;
        let positions: Vec<_> = particles
            .iter()
            .map(|p| p.position)
            .chain(vec![vector![0.0, 0.0]; additional_unused_particles])
            .collect::<Vec<_>>();
        let dynamics: Vec<_> = particles
            .iter()
            .map(|p| p.dynamics)
            .chain(vec![
                ParticleDynamics::with_density(0.0, 0.0);
                additional_unused_particles
            ])
            .collect();
        Self {
            positions: GpuVector::init(
                device,
                &positions,
                BufferUsages::STORAGE | BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
            ),
            dynamics: GpuVector::encase(
                device,
                &dynamics,
                BufferUsages::STORAGE | BufferUsages::COPY_DST,
            ),
            sorted_ids: GpuVector::uninit(
                device,
                maximum_size as u32,
                BufferUsages::STORAGE | BufferUsages::COPY_DST,
            ),
            node_linked_lists: GpuVector::uninit(
                device,
                maximum_size as u32,
                BufferUsages::STORAGE | BufferUsages::COPY_DST,
            ),
            current_size: GpuScalar::init(
                device,
                particles_len as u32,
                BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            ),
            current_size_cached: particles_len as u32,
            maximum_size: maximum_size as u32,
        }
    }

    // TODO: push multiple particles at once.
    /// This is designed to work in conjunction with [`crate::models::GpuModels::push`]
    pub fn push(&mut self, queue: &wgpu::Queue, particle: &Particle) {
        if self.maximum_size == self.current_size_cached {
            eprintln!("Particle buffer is full.");
            return;
        }
        println!(
            "ParticleDynamics min_size: {} bytes",
            ParticleDynamics::min_size().get()
        );
        println!("Cdf min_size: {} bytes", Cdf::min_size().get());
        println!("pushed a new particle!");
        // TODO: provide a helper in Tensor to push values?

        // Push position
        let offset = self.current_size_cached as u64 * Vector2::<f32>::min_size().get();
        let position = particle.position;
        // Turn value into &[u8] (raw bytes)
        let bytes = bytemuck::bytes_of(&position);
        queue.write_buffer(&self.positions.buffer(), offset, bytes);

        // Push dynamics.
        let offset = self.current_size_cached as u64 * ParticleDynamics::min_size().get();
        let dynamics = particle.dynamics;
        // Turn value into &[u8] (raw bytes)
        let mut buffer = encase::StorageBuffer::new(Vec::<u8>::new());
        buffer.write(&dynamics).unwrap();
        let bytes = buffer.as_ref();
        queue.write_buffer(&self.dynamics.buffer(), offset, bytes);

        self.current_size_cached += 1;
        queue.write_buffer(
            &self.current_size.buffer(),
            0,
            bytemuck::bytes_of(&self.current_size_cached),
        );
    }
}

#[derive(Shader)]
#[shader(src = "particle2d.wgsl", shader_defs = "dim_shader_defs")]
pub struct WgParticle;

#[derive(Copy, Clone, Debug, ShaderType)]
pub struct GpuSampleIds {
    pub segment: Vector2<u32>,
    pub collider: u32,
}

#[derive(Copy, Clone, Debug)]
struct SamplingParams {
    base_vid: u32,
    collider_id: u32,
    sampling_step: f32,
}

#[derive(Default, Clone)]
struct SamplingBuffers {
    samples: Vec<Point2<f32>>,
    samples_ids: Vec<GpuSampleIds>,
}

// TODO: move this elsewhere?
fn sample_polyline(polyline: &Polyline, params: &SamplingParams, buffers: &mut SamplingBuffers) {
    for seg_idx in polyline.indices() {
        let seg = Segment::new(
            polyline.vertices()[seg_idx[0] as usize],
            polyline.vertices()[seg_idx[1] as usize],
        );
        let sample_id = GpuSampleIds {
            segment: vector![params.base_vid + seg_idx[0], params.base_vid + seg_idx[1]],
            collider: params.collider_id,
        };
        buffers.samples.push(seg.a);
        buffers.samples_ids.push(sample_id);

        if let Some(dir) = seg.direction() {
            for i in 0.. {
                let shift = (i as f32) * params.sampling_step;
                if shift > seg.length() {
                    break;
                }

                buffers.samples.push(seg.a + *dir * shift);
                buffers.samples_ids.push(sample_id);
            }

            buffers.samples.push(seg.b);
            buffers.samples_ids.push(sample_id);
        }
    }
}

wgcore::test_shader_compilation!(WgParticle);
