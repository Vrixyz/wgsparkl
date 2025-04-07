use bevy::prelude::*;
use wgsparkl_testbed3d::{
    init_testbed,
    load_scene::{SceneInitFn, SceneInits, SceneLoadFn},
};

pub mod utils;

mod banana3;
mod elastic_cut3;
mod glb_to_point_cloud_color;
mod heightfield3;
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
    let scenes: Vec<(String, SceneInitFn, Option<SceneLoadFn>)> = vec![
        ("sand".to_string(), Box::new(sand3::sand_demo), None),
        (
            "heightfield".to_string(),
            Box::new(heightfield3::heightfield_demo),
            None,
        ),
        (
            "elastic_cut".to_string(),
            Box::new(elastic_cut3::elastic_cut_demo),
            None,
        ),
        (
            "elastic_model_colors".to_string(),
            Box::new(glb_to_point_cloud_color::elastic_color_model_demo),
            Some(Box::new(glb_to_point_cloud_color::load)),
        ),
        (
            "taichi_banana".to_string(),
            Box::new(banana3::demo),
            Some(Box::new(banana3::load)),
        ),
    ];
    let mut inits = world.resource_mut::<SceneInits>();
    inits.scenes = scenes;
}

fn start_default_scene(mut commands: Commands, scenes: Res<SceneInits>) {
    scenes.init_scene(&mut commands, 0);
}
