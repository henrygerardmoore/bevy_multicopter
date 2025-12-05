use bevy::prelude::*;
use serde::{Deserialize, Serialize};

pub struct QuadcopterDerivatives {
    pub acceleration: Vec3,
    pub angular_acceleration: Vec3,
}

#[derive(Serialize, Deserialize, Reflect)]
pub struct PropellerInfo {
    /// the body frame position of this propeller
    position: Vec3,
    direction: Vec3,
    thrust_constant: f32,
    drag_constant: f32,
}

#[derive(Component, Serialize, Deserialize, Reflect)]
#[reflect(Component)]
pub struct Multicopter {
    inverse_inertia: Mat3,
    /// the 3x3 inertia matrix, each element in kg * m^2
    inertia: Mat3,
    /// the inverse mass of the quadcopter, 1/kg
    inverse_mass: f32,
    propellers: Vec<PropellerInfo>,
}

impl Multicopter {
    /// the state derivative of this quadcopter given its current world transform and the control inputs
    /// equations adapted from [Modelling and control of quadcopter](https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)
    pub fn state_derivative(
        &self,
        quadcopter_state: GlobalTransform,
        angular_velocity: Vec3,
        // TODO: instead of inputting omega directly, allow for thrust curves or something
        quadcopter_control_inputs: Vec<f32>,
        external_force: Vec3,
        external_torque: Vec3,
    ) -> Result<QuadcopterDerivatives, String> {
        if quadcopter_control_inputs.len() != self.propellers.len() {
            return Err("Incorrect control input length".into());
        }

        // TODO: add aerodynamic effects

        // the force of each prop
        let forces: Vec<_> = self
            .propellers
            .iter()
            .zip(quadcopter_control_inputs.iter())
            .map(|(propeller, omega)| {
                propeller.thrust_constant * omega.powi(2) * propeller.direction
            })
            .collect();

        // the net thrust vector of the 'copter
        let thrust: Vec3 = forces.iter().cloned().sum();

        // the net torque due to the propellers
        let propeller_torque: Vec3 = self
            .propellers
            .iter()
            .zip(forces)
            .zip(quadcopter_control_inputs)
            .map(|((propeller, force), omega)| {
                // torque due to thrust of propeller
                propeller.position.cross(force)
            // torque due to rotational drag of propeller
            + propeller.drag_constant * omega.powi(2)
            })
            .sum();

        let acceleration =
            self.inverse_mass * (external_force + quadcopter_state.rotation() * thrust);
        let angular_acceleration = self.inertia.inverse()
            * (external_torque + quadcopter_state.rotation() * propeller_torque
                - angular_velocity.cross(self.inertia * angular_velocity));
        Ok(QuadcopterDerivatives {
            acceleration,
            angular_acceleration,
        })
    }

    pub fn new(inertia: Mat3, mass: f32, propellers: Vec<PropellerInfo>) -> Result<Self, String> {
        assert!(!propellers.is_empty(), "Don't try to simulate a 0-copter");
        let inverse_inertia = inertia.inverse();
        Ok(Self {
            inertia,
            inverse_mass: mass.recip(),
            propellers,
            inverse_inertia,
        })
    }
}

// TODO: add unit tests comparing hand-calculations to `state_derivative`
