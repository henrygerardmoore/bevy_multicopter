use avian3d::prelude::*;
use bevy::{core_pipeline::Skybox, prelude::*};
use bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_quadcopter::{Multicopter, PropellerInfo, RotationDirection};

fn main() {
    App::new()
        // add an ambient light so it doesn't look so stark
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0 / 5.0f32,
            ..default()
        })
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            EguiPlugin::default(),
            WorldInspectorPlugin::new(),
            PhysicsDebugPlugin::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, (control_quadcopter, change_window_title))
        .add_systems(FixedUpdate, (quadcopter_dynamics, camera_follow_quadcopter))
        .run();
}

fn change_window_title(mut window: Single<&mut Window>) {
    window.title = "Quadcopter Simulation".into();
}

// this value gives ~10k RPM at hover, which seems reasonable
const THRUST_CONSTANT: f32 = 1e-6;
const DRAG_CONSTANT: f32 = 1e-7;

// helper for this quad's propellers
fn propeller_from_position(x: f32, y: f32, z: f32, direction: RotationDirection) -> PropellerInfo {
    PropellerInfo {
        position: Vec3 { x, y, z },
        direction: Dir3::new(Vec3::Y).unwrap(),
        thrust_constant: THRUST_CONSTANT,
        drag_constant: DRAG_CONSTANT,
        rotation_direction: direction,
    }
}

pub fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut game_time: ResMut<Time<Virtual>>,
    gravity: Res<Gravity>,
) {
    game_time.pause();
    let skybox_handle = asset_server.load("skybox.ktx2");
    // spawn the game camera
    commands.spawn((
        Name::new("camera"),
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 90.0_f32.to_radians(),
            ..default()
        }),
        Skybox {
            image: skybox_handle,
            brightness: 1000.0,
            ..default()
        },
    ));

    // spawn lights
    commands.spawn((
        Name::new("Sun"),
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_translation(Vec3 {
            x: 3.,
            y: 10.,
            z: 1.,
        })
        .looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let propellers = vec![
        propeller_from_position(0.05, 0., 0.05, RotationDirection::CounterClockWise),
        propeller_from_position(-0.05, 0., 0.05, RotationDirection::ClockWise),
        propeller_from_position(0.05, 0., -0.05, RotationDirection::ClockWise),
        propeller_from_position(-0.05, 0., -0.05, RotationDirection::CounterClockWise),
    ];

    let quad_mass = 0.4; // kg
    let g = gravity.0.y.abs();
    // counteract the force of gravity, m * g
    let hover_thrust_per_prop = quad_mass * g / 4.;

    // thrust = k * omega^2
    let hover_omega = (hover_thrust_per_prop / THRUST_CONSTANT).sqrt();
    // spawn a quadcopter
    commands.spawn((
        Name::new("quadcopter"),
        SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("quadcopter.glb"))),
        RigidBody::Dynamic,
        Transform::from_xyz(0., 2., 0.),
        // it's fine for the fine collision details of the quad to be lost
        Collider::cuboid(0.1, 0.04, 0.1),
        QuadcopterControls([hover_omega; 4]),
        Multicopter::new(propellers).unwrap(),
        Mass(quad_mass),
        AngularInertia::new(Vec3::splat(1e-2)),
        SweptCcd::NON_LINEAR,
        AngularDamping(0.5),
        LinearDamping(0.5),
    ));

    // spawn the level
    commands.spawn((
        Name::new("level"),
        SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("level.glb"))),
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),
        RigidBody::Static,
    ));
}

#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct QuadcopterControls([f32; 4]);

// control the quad with keyboard inputs
pub fn control_quadcopter(
    keyboard_inputs: Res<ButtonInput<KeyCode>>,
    mut quadcopter_query: Single<&mut QuadcopterControls, With<Multicopter>>,
    mut time: ResMut<Time<Virtual>>,
    mut quadcopter_transform_query: Single<
        (
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &ComputedMass,
        ),
        With<Multicopter>,
    >,
    gravity: Res<Gravity>,
    mut desired_altitude: Local<Option<f32>>,
    mut integral_term: Local<f32>,
) {
    // TODO: unpause on assets loaded instead of manually doing it
    if keyboard_inputs.just_pressed(KeyCode::KeyP) {
        if time.is_paused() {
            time.unpause();
        } else {
            time.pause();
        }
    }
    // reset copter position
    if keyboard_inputs.just_pressed(KeyCode::KeyR) {
        println!("Resetting quadcopter");
        *quadcopter_transform_query.0 = Transform::from_xyz(0., 2., 0.);
        *quadcopter_transform_query.1 = LinearVelocity(Vec3::ZERO);
        *quadcopter_transform_query.2 = AngularVelocity(Vec3::ZERO);
    }
    let (transform, linear_velocity, angular_velocity, mass) =
        quadcopter_transform_query.into_inner();

    // transform angular velocity to body frame
    let angular_velocity = transform.rotation.inverse() * angular_velocity.0;

    // on first run, set the desired altitude to our current altitude
    if desired_altitude.is_none() {
        *desired_altitude = Some(transform.translation.y);
    }
    let dt = time.delta_secs();

    let (yaw, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);
    let mut desired_pitch = 0.;
    let mut desired_roll = 0.;
    // we control yaw rate instead of yaw, so we always set the desired yaw to the current
    let desired_yaw = yaw;
    let mut desired_yaw_rate = 0.;

    if keyboard_inputs.pressed(KeyCode::KeyW) {
        desired_pitch -= 20.0_f32.to_radians();
    }
    if keyboard_inputs.pressed(KeyCode::KeyS) {
        desired_pitch += 20.0_f32.to_radians();
    }

    if keyboard_inputs.pressed(KeyCode::KeyD) {
        desired_roll -= 20.0_f32.to_radians();
    }
    if keyboard_inputs.pressed(KeyCode::KeyA) {
        desired_roll += 20.0_f32.to_radians();
    }

    let desired_altitude = desired_altitude.as_mut().unwrap();
    if keyboard_inputs.pressed(KeyCode::Space) {
        *desired_altitude += dt * 2.;
    }
    if keyboard_inputs.pressed(KeyCode::ControlLeft) {
        *desired_altitude -= dt * 2.;
    }

    if keyboard_inputs.pressed(KeyCode::KeyQ) {
        desired_yaw_rate += 1.;
    }
    if keyboard_inputs.pressed(KeyCode::KeyE) {
        desired_yaw_rate -= 1.;
    }

    // target hover
    let vertical_proportional_gain = 40.;
    let vertical_derivative_gain = 30.;
    let vertical_i_gain = 0.;
    let gravity_force = gravity.0.y * mass.value();
    let altitude = transform.translation.y;
    let vertical_velocity = linear_velocity.y;
    let p_term = vertical_proportional_gain * (*desired_altitude - altitude);
    let d_term = vertical_derivative_gain * (0. - vertical_velocity);
    *integral_term += dt * (*desired_altitude - altitude);
    let i_term = vertical_i_gain * *integral_term;
    let desired_vertical_thrust = p_term + i_term + d_term - gravity_force;

    // the proportion of thrust that actually helps go up
    let vertical_thrust_coeff = transform.local_y().dot(Vec3::Y);
    let needed_vertical_thrust = if vertical_thrust_coeff <= 1e-2 || desired_vertical_thrust < 0. {
        // if we are barely pointing up or pointing down (or our desired thrust is negative), then just accept that we can't produce the necessary vertical thrust
        0.
    } else {
        desired_vertical_thrust / vertical_thrust_coeff
    };

    // the portion of the whole that each propeller must take
    // they will sum to 1. and thus achieve the necessary thrust, but may take on different values
    // in order to achieve the necessary angle control
    let mut propeller_thrust_proportions = [0.0_f32; 4];

    let angular_proportional_gain = 0.05;
    let angular_derivative_gain = 0.01;
    let roll_diff = desired_roll - roll;
    let pitch_diff = desired_pitch - pitch;
    let yaw_diff = desired_yaw - yaw;
    let angular_velocity_difference = Vec3 {
        x: 0.,
        y: desired_yaw_rate,
        z: 0.,
    } - angular_velocity;

    let p_pitch = angular_proportional_gain * (pitch_diff);
    let d_pitch = angular_derivative_gain * angular_velocity_difference.x;
    let desired_pitch_torque = p_pitch + d_pitch;

    let p_yaw = angular_proportional_gain * (yaw_diff);
    let d_yaw = angular_derivative_gain * angular_velocity_difference.y;
    let desired_yaw_torque = p_yaw + d_yaw;

    let p_roll = angular_proportional_gain * (roll_diff);
    let d_roll = angular_derivative_gain * angular_velocity_difference.z;
    let desired_roll_torque = p_roll + d_roll;

    // map the desired roll pitch yaw torques to the propellers according to their positions
    propeller_thrust_proportions[0] =
        -desired_pitch_torque + desired_roll_torque + desired_yaw_torque;
    propeller_thrust_proportions[1] =
        -desired_pitch_torque - desired_roll_torque - desired_yaw_torque;
    propeller_thrust_proportions[2] =
        desired_pitch_torque + desired_roll_torque - desired_yaw_torque;
    propeller_thrust_proportions[3] =
        desired_pitch_torque - desired_roll_torque + desired_yaw_torque;

    // compute the values such that no propeller wants to go backwards (may result in less control authority)
    let reduction_factor =
        propeller_thrust_proportions
            .iter()
            .cloned()
            .fold(1.0_f32, |acc, proportion| {
                acc.min(if proportion < -0.25 {
                    -0.25 / proportion
                } else {
                    1.
                })
            });

    // shrink commands down if necessary and add the 0.25 baseline corresponding to equal control of each
    let propeller_thrust_proportions =
        propeller_thrust_proportions.map(|proportion| 0.25 + proportion * reduction_factor);

    let controls = quadcopter_query.as_mut();
    let necessary_propeller_rotation_rate = (needed_vertical_thrust / THRUST_CONSTANT).sqrt();
    controls.0 = propeller_thrust_proportions
        .map(|proportion| necessary_propeller_rotation_rate * proportion);
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

    let camera_position = quad_tf.translation() + -0.5 * quad_tf.forward() + 0.25 * quad_tf.up();
    camera_tf.translation = camera_position;
    *camera_tf = camera_tf.looking_to(quad_tf.forward(), quad_tf.up());
}
