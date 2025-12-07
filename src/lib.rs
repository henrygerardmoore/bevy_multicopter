use bevy::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub struct QuadcopterForceTorque {
    pub force: Vec3,
    pub torque: Vec3,
}

#[derive(Serialize, Deserialize, Reflect)]
pub enum RotationDirection {
    CounterClockWise,
    ClockWise,
}

#[derive(Serialize, Deserialize, Reflect)]
pub struct PropellerInfo {
    /// the body frame position of this propeller
    pub position: Vec3,
    pub direction: Dir3,
    pub thrust_constant: f32,
    pub drag_constant: f32,
    pub rotation_direction: RotationDirection,
}

#[derive(Component, Serialize, Deserialize, Reflect)]
#[reflect(Component)]
pub struct Multicopter {
    propellers: Vec<PropellerInfo>,
}

impl Multicopter {
    /// the state derivative of this quadcopter given its current world transform and the control inputs
    /// equations adapted from [Modelling and control of quadcopter](https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)
    pub fn force_torque(
        &self,
        quadcopter_state: &GlobalTransform,
        angular_velocity: &Vec3,
        // TODO: instead of inputting omega directly, allow for thrust curves or something
        quadcopter_control_inputs: &Vec<f32>,
        inertia: &Mat3,
    ) -> Result<QuadcopterForceTorque, String> {
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
            + propeller.drag_constant * omega.powi(2) * match propeller.rotation_direction {
                RotationDirection::CounterClockWise => 1.,
                RotationDirection::ClockWise => -1.,
            }
            })
            .sum();

        let force = quadcopter_state.rotation() * thrust;
        let torque = quadcopter_state.rotation() * propeller_torque
            - angular_velocity.cross(inertia * angular_velocity);
        Ok(QuadcopterForceTorque { force, torque })
    }

    pub fn new(propellers: Vec<PropellerInfo>) -> Result<Self, String> {
        assert!(!propellers.is_empty(), "Don't try to simulate a 0-copter");
        Ok(Self { propellers })
    }
}

// TODO: add unit tests comparing hand-calculations to `state_derivative`
