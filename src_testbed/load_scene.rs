use bevy::ecs::system::SystemId;
use bevy::prelude::*;
use bevy::render::renderer::RenderDevice;

use crate::file_loader::{FileBytes, FileBytesLoader};
use crate::*;

pub fn load_scene_plugin(app: &mut App) {
    app.init_resource::<SceneInits>();

    app.init_state::<SceneState>();
    app.init_asset::<FileBytes>();
    app.init_asset_loader::<FileBytesLoader>();
    app.add_systems(Update, check_scene_loaded);
    app.add_systems(OnEnter(SceneState::Loaded), call_init_state);
}

#[derive(Resource, Default)]
pub struct CurrentSceneId(pub usize);

#[derive(Copy, Clone, States, Hash, PartialEq, Eq, Debug, Default)]
pub enum SceneState {
    #[default]
    Waiting,
    Loading,
    Loaded,
}

#[derive(Resource)]
pub struct SceneInits {
    pub scenes: Vec<(String, SceneInitFn, Option<SceneLoadFn>)>,
    reset_graphics: SystemId,
    reset_app_state: SystemId,
}
pub type SceneInitFn = Box<
    dyn FnMut(RenderDevice, &mut AppState, &mut Callbacks, &Dependencies) -> PhysicsContext
        + Send
        + Sync,
>;

pub type SceneLoadFn = Box<dyn FnMut(&mut Loader) + Send + Sync>;

impl SceneInits {
    pub fn init_scene(&self, commands: &mut Commands, scene_id: usize) {
        commands.run_system(self.reset_app_state);
        commands.set_state(SceneState::Loading);

        commands.run_system_cached_with(start_scene_loading, scene_id);
    }
    pub fn on_scene_loaded(&self, commands: &mut Commands, scene_id: usize) {
        commands.run_system_cached_with(run_scene_init, scene_id);
        commands.run_system(self.reset_graphics);
        commands.set_state(SceneState::Loaded);
    }
}

pub fn check_scene_loaded(
    mut next_scene_state: ResMut<NextState<SceneState>>,
    loading_assets: Res<LoadingAssets>,
    asset_server: Res<AssetServer>,
) {
    if loading_assets
        .assets
        .iter()
        .any(|a| !asset_server.is_loaded_with_dependencies(a.1))
    {
        return;
    }
    next_scene_state.set(SceneState::Loaded);
}
pub fn call_init_state(
    mut commands: Commands,
    current_scene_id: Res<CurrentSceneId>,
    scene_inits: Res<SceneInits>,
) {
    scene_inits.on_scene_loaded(&mut commands, current_scene_id.0);
}

pub fn start_scene_loading(scene_id: In<usize>, world: &mut World) {
    world.get_resource_or_init::<LoadingAssets>().assets.clear();
    world.insert_resource(CurrentSceneId(scene_id.0));
    let mut scenes = world.remove_resource::<SceneInits>().unwrap();
    let mut loader = Loader::new(world);
    if let Some(scene_load) = scenes.scenes[scene_id.0].2.as_mut() {
        scene_load(&mut loader);
    }
    world.insert_resource(scenes);
}

pub fn run_scene_init(
    scene_id: In<usize>,
    mut commands: Commands,
    mut scenes: ResMut<SceneInits>,
    device: Res<RenderDevice>,
    mut app_state: ResMut<AppState>,
    mut callbacks: ResMut<Callbacks>,
    file_bytes: Res<Assets<FileBytes>>,
    loading_assets: Res<LoadingAssets>,
) {
    commands.insert_resource(scenes.scenes[scene_id.0].1(
        device.clone(),
        &mut app_state,
        &mut callbacks,
        &Dependencies {
            assets: &loading_assets,
            source: &file_bytes,
        },
    ));
}

impl FromWorld for SceneInits {
    fn from_world(world: &mut World) -> Self {
        Self {
            scenes: vec![],
            reset_graphics: world.register_system(startup::setup_graphics),
            reset_app_state: world.register_system(startup::setup_app_state),
        }
    }
}

#[derive(Resource, Default)]
pub struct LoadingAssets {
    pub assets: HashMap<String, Handle<FileBytes>>,
}

pub struct Dependencies<'a, 's> {
    assets: &'a LoadingAssets,
    source: &'s Assets<FileBytes>,
}

impl<'a, 's> Dependencies<'a, 's> {
    pub fn get_data(&self, path: &str) -> Option<&FileBytes> {
        let handle = self.assets.assets.get(path)?;
        self.source.get(handle)
    }
}

pub struct Loader<'w> {
    world: &'w mut World,
}

impl<'w> Loader<'w> {
    pub fn new(world: &'w mut World) -> Self {
        Self { world }
    }

    pub fn load_raw_file(&mut self, path: &str) -> Handle<FileBytes> {
        let handle = self.world.load_asset(path);
        let mut loading_assets = self.world.get_resource_or_init::<LoadingAssets>();
        loading_assets
            .assets
            .insert(path.to_string(), handle.clone());
        handle
    }
}
