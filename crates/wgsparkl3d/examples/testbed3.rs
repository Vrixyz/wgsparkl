use bevy::prelude::*;
use wgsparkl_testbed3d::{init_testbed, SceneInits};

mod banana3;
mod elastic_cut3;
mod glb_to_point_cloud_color;
mod heightfield3;
mod obj_to_point_cloud;
mod sand3;

pub fn main() {
    let mut app = App::new();
    init_testbed(&mut app);
    app.add_systems(
        Startup,
        (register_scenes, start_default_scene)
            .chain()
            .after(wgsparkl_testbed3d::startup::setup_app),
    );
    app.run();
}

fn register_scenes(world: &mut World) {
    let scenes = vec![
        ("sand".to_string(), world.register_system(sand3::sand_demo)),
        (
            "heightfield".to_string(),
            world.register_system(heightfield3::heightfield_demo),
        ),
        (
            "elastic_cut".to_string(),
            world.register_system(elastic_cut3::elastic_cut_demo),
        ),
        (
            "elastic_model".to_string(),
            world.register_system(obj_to_point_cloud::elastic_model_demo),
        ),
        (
            "elastic_model_colors".to_string(),
            world.register_system(glb_to_point_cloud_color::elastic_color_model_demo),
        ),
        (
            "taichi_banana".to_string(),
            world.register_system(banana3::demo),
        ),
    ];
    let mut inits = world.resource_mut::<SceneInits>();
    inits.scenes = scenes;
}

fn start_default_scene(mut commands: Commands, scenes: Res<SceneInits>) {
    scenes.init_scene(&mut commands, 0);
}
