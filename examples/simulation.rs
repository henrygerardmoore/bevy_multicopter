use avian3d::prelude::*;
use bevy::{app::FixedMain, prelude::*};
use bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_quadcopter::{Multicopter, PropellerInfo};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            EguiPlugin::default(),
            WorldInspectorPlugin::new(),
        ))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                control_quadcopter.before(camera_follow_quadcopter),
                camera_follow_quadcopter,
            ),
        )
        .add_systems(FixedMain, quadcopter_dynamics)
        .run();
}

pub fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // spawn the game camera
    commands.spawn((Name::new("camera"), Camera3d::default()));

    let propellers = vec![
        PropellerInfo::from_position(Vec3 {
            x: 0.05,
            y: 0.,
            z: 0.05,
        }),
        PropellerInfo::from_position(Vec3 {
            x: -0.05,
            y: 0.,
            z: 0.05,
        }),
        PropellerInfo::from_position(Vec3 {
            x: 0.05,
            y: 0.,
            z: -0.05,
        }),
        PropellerInfo::from_position(Vec3 {
            x: -0.05,
            y: 0.,
            z: -0.05,
        }),
    ];

    // spawn a quadcopter
    commands.spawn((
        Name::new("quadcopter"),
        SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("quadcopter.glb"))),
        RigidBody::Dynamic,
        Transform::from_xyz(0., 2., 0.),
        // it's fine for the fine collision details of the quad to be lost
        ColliderConstructorHierarchy::new(ColliderConstructor::ConvexHullFromMesh),
        // TODO: remove when there is a level floor
        GravityScale(0.),
        QuadcopterControls([0.; 4]),
        Multicopter::new(propellers).unwrap(),
        Mass(0.1),
        AngularInertia::new(Vec3 {
            x: 0.01,
            y: 0.01,
            z: 0.01,
        }),
    ));

    // TODO: spawn a level
}

#[derive(Component)]
pub struct QuadcopterControls([f32; 4]);

// control the quad with keyboard inputs
pub fn control_quadcopter(
    _keyboard_inputs: Res<ButtonInput<KeyCode>>,
    _quadcopter_query: Query<(&mut QuadcopterControls, &Multicopter)>,
) {
    // TODO: map keyboard input to desired angular and linear vel
    // TODO: do simple PD to command quad and compensate for gravity
}

pub fn quadcopter_dynamics(
    quadcopter_query: Query<(
        &Multicopter,
        &GlobalTransform,
        &QuadcopterControls,
        &AngularInertia,
        Forces,
    )>,
    time: Res<Time<Virtual>>,
) {
    let dt = time.delta_secs();
    for (multicopter, transform, controls, inertia, mut forces) in quadcopter_query {
        let Ok(force_torque) = multicopter
            .force_torque(
                transform,
                &forces.angular_velocity(),
                &controls.0.iter().cloned().collect(),
                &inertia.tensor().to_mat3(),
            )
            .map_err(|err| {
                error!(err);
            })
        else {
            continue;
        };
        forces.apply_linear_impulse(dt * force_torque.force);
        forces.apply_angular_impulse(dt * force_torque.torque);
    }
}

pub fn camera_follow_quadcopter(
    quadcopter_query: Single<&GlobalTransform, With<Multicopter>>,
    mut camera_query: Single<&mut Transform, With<Camera>>,
) {
    // move the camera to behind the quadcopter
    let quad_tf = quadcopter_query.into_inner();
    let camera_tf = camera_query.as_mut();

    let camera_position = quad_tf.translation() + -1. * quad_tf.forward() + 0.5 * quad_tf.up();
    camera_tf.translation = camera_position;
    camera_tf.look_at(quad_tf.translation(), quad_tf.up());
}
