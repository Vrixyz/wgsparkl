#define_import_path wgsparkl::examples::prep_vertex_buffer

#import wgsparkl::solver::particle as Particle;
#import wgsparkl::grid::grid as Grid;
#import wgsparkl::solver::params as Params;
#import wgebra::svd2 as Svd2;

@group(0) @binding(0)
var<storage, read_write> instances: array<InstanceData>;
@group(0) @binding(1)
var<storage, read> particles_pos: array<Particle::Position>;
@group(0) @binding(2)
var<storage, read> particles_vol: array<Particle::Volume>;
@group(0) @binding(3)
var<storage, read> particles_vel: array<Particle::Velocity>;
@group(0) @binding(4)
var<storage, read_write> grid: Grid::Grid;
@group(0) @binding(5)
var<uniform> params: Params::SimulationParams;
@group(0) @binding(6)
var<storage, read> config: RenderConfig;

struct RenderConfig {
    mode: u32,
}

const DEFAULT: u32 = 0;
const VOLUME: u32 = 1;
const VELOCITY: u32 = 2;

struct InstanceData {
    deformation: mat3x3<f32>,
    position: vec3<f32>,
    base_color: vec4<f32>,
    color: vec4<f32>,
}

@compute @workgroup_size(64, 1, 1)
fn main(
    @builtin(global_invocation_id) tid: vec3<u32>,
) {
    let particle_id = tid.x;

    if particle_id < arrayLength(&instances) {
        let def_grad = Particle::deformation_gradient(particles_vol[particle_id]);
        instances[particle_id].deformation = mat3x3(vec3(def_grad.x, 0.0), vec3(def_grad.y, 0.0), vec3(0.0, 0.0, 1.0));
        instances[particle_id].position = vec3(particles_pos[particle_id].pt, 0.0);

        let color = instances[particle_id].base_color;
        let cell_width = grid.cell_width;
        let dt = params.dt;
        let max_vel = cell_width / dt;

        if config.mode == DEFAULT {
            instances[particle_id].color = color;
        } else if config.mode == VELOCITY {
            let vel = particles_vel[particle_id].v;
            instances[particle_id].color = vec4(abs(vel) * dt * 100.0 + vec2(0.2), color.z, color.w);
        } else if config.mode == VOLUME {
            let svd = Svd2::svd(def_grad);
            let color_xy = (vec2(1.0) - svd.S) / 0.005 + vec2(0.2);
            instances[particle_id].color = vec4(color_xy, color.z, color.w);
        }
    }
}