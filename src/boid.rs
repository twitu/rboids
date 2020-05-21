use ncollide3d::nalgebra::{Point3, Vector3, Vector4};
use ncollide3d::nalgebra::geometry::UnitQuaternion;

// Scaling factors
const TIME_SCALE: f32 = 1.0;

/// boid struct is a point with a velocity
#[derive(Clone)]
pub struct Boid {
    pos: Point3<f32>,
    vel: Vector3<f32>,
}

impl Boid {
    /// create new boid
    pub fn new(pos: Vec<f32>, vel: Vec<f32>) -> Self {
        Boid {
            pos: Point3::from(Vector3::from_vec(pos)),
            vel: Vector3::from_vec(vel),
        }
    }

    /// return position as array
    pub fn pos_array(&self) -> [f32; 3] {
        self.pos.coords.into()
    }

    /// return velocity as array
    pub fn vel_array(&self) -> [f32; 3] {
        self.vel.into()
    }

    /// return rotation from y-axis towards velocity
    pub fn rot_array(&self) -> [f32; 4] {
        let rot: Vector4<f32> = *UnitQuaternion::rotation_between(&Vector3::y_axis(), &self.vel)
            .unwrap_or(UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::PI),)
            .as_vector();
        rot.into()
    }

    pub fn frame_update(&mut self, delta_time: f32) {
        // update position
        self.pos += self.vel * delta_time * TIME_SCALE;
    }
}
