use ncollide3d::nalgebra::{Point3, Vector3};

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

    pub fn frame_update(&mut self, delta_time: f32) {
        // update position
        self.pos += self.vel * delta_time * TIME_SCALE;
    }
}
