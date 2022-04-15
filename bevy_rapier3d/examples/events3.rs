extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use rapier3d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_startup_system(enable_physics_profiling)
        .add_system_to_stage(CoreStage::PostUpdate, display_events)
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(mut commands: Commands) {
    const HALF_SIZE: f32 = 100.0;

    commands.spawn_bundle(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            // Configure the projection to better fit the scene
            shadow_projection: OrthographicProjection {
                left: -HALF_SIZE,
                right: HALF_SIZE,
                bottom: -HALF_SIZE,
                top: HALF_SIZE,
                near: -10.0 * HALF_SIZE,
                far: 100.0 * HALF_SIZE,
                ..Default::default()
            },
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform {
            translation: Vec3::new(10.0, 2.0, 10.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_4),
            ..Default::default()
        },
        ..Default::default()
    });
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::look_at_rh(
            Vec3::new(0.0, 0.0, 25.0),
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )),
        ..Default::default()
    });
}

fn display_events(
    mut intersection_events: EventReader<IntersectionEvent>,
    mut contact_events: EventReader<ContactEvent>,
) {
    for intersection_event in intersection_events.iter() {
        println!("Received intersection event: {:?}", intersection_event);
    }

    for contact_event in contact_events.iter() {
        println!("Received contact event: {:?}", contact_event);
    }
}

pub fn setup_physics(mut commands: Commands) {
    let parent = commands
        .spawn_bundle((Transform::identity(), GlobalTransform::identity()))
        .id();

    /*
     * Ground
     */
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(4.0, 1.2, 1.2).into(),
        ..ColliderBundle::default()
    };

    commands
        .spawn_bundle(collider)
        .insert(Parent(parent))
        .insert(ColliderDebugRender::default())
        .insert(ColliderPositionSync::Discrete);

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(4.0, 1.2, 1.0).into(),
        collider_type: ColliderType::Sensor.into(),
        position: [0.0, 5.0, 0.0].into(),
        ..ColliderBundle::default()
    };

    commands
        .spawn_bundle(collider)
        .insert(ColliderDebugRender::with_id(0))
        .insert(ColliderPositionSync::Discrete);

    let rigid_body = RigidBodyBundle {
        position: [0.0, 13.0, 0.0].into(),
        ..RigidBodyBundle::default()
    };

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(0.5, 0.5, 0.5).into(),
        flags: (ActiveEvents::INTERSECTION_EVENTS | ActiveEvents::CONTACT_EVENTS).into(),
        ..ColliderBundle::default()
    };

    commands
        .spawn_bundle(rigid_body)
        .insert_bundle(collider)
        .insert(ColliderDebugRender::with_id(1))
        .insert(ColliderPositionSync::Discrete);
}
