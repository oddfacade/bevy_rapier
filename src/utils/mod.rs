use bevy::prelude::{GlobalTransform, Transform};
use bevy::math::{Affine3A, Mat3A};
use rapier::math::{Isometry, Real};

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim2")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    Transform {
        translation: (iso.translation.vector.push(0.0) * physics_scale).into(),
        rotation: bevy::prelude::Quat::from_axis_angle(
            bevy::prelude::Vec3::Z,
            iso.rotation.angle(),
        ),
        ..Default::default()
    }
}

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim3")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    Transform {
        translation: (iso.translation.vector * physics_scale).into(),
        rotation: iso.rotation.into(),
        ..Default::default()
    }
}

/// Converts a Rapier isometry to a Bevy global transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim2")]
pub fn iso_to_global_transform(iso: &Isometry<Real>, physics_scale: Real) -> GlobalTransform {
    Affine3A {
        translation: (iso.translation.vector.push(0.0) * physics_scale).into(),
        matrix3: Mat3A::from_axis_angle(
            bevy::prelude::Vec3::Z,
            iso.rotation.angle(),
        ),
        ..Default::default()
    }.into()
}

/// Converts a Rapier isometry to a Bevy global transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim3")]
pub fn iso_to_global_transform(iso: &Isometry<Real>, physics_scale: Real) -> GlobalTransform {
    Affine3A {
        translation: (iso.translation.vector * physics_scale).into(),
        matrix3: Mat3A::from_quat(iso.rotation.into()),
        ..Default::default()
    }.into()
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim2")]
#[allow(dead_code)]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    use bevy::math::Vec3Swizzles;
    Isometry::new(
        (transform.translation / physics_scale).xy().into(),
        transform.rotation.to_scaled_axis().z,
    )
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim3")]
#[allow(dead_code)]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    Isometry::new(
        (transform.translation / physics_scale).into(),
        transform.rotation.to_scaled_axis().into(),
    )
}

/// Converts a Bevy global transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim2")]
pub(crate) fn global_transform_to_iso(transform: &GlobalTransform, physics_scale: Real) -> Isometry<Real> {
    use bevy::math::Vec3Swizzles;
    let (_scale, rotation, translation) = transform.to_scale_rotation_translation();
    Isometry::new(
        (translation / physics_scale).xy().into(),
        rotation.to_scaled_axis().z,
    )
}

/// Converts a Bevy global transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim3")]
pub(crate) fn global_transform_to_iso(transform: &GlobalTransform, physics_scale: Real) -> Isometry<Real> {
    let (_scale, rotation, translation) = transform.to_scale_rotation_translation();
    Isometry::new(
        (translation / physics_scale).into(),
        rotation.to_scaled_axis().into(),
    )
}
