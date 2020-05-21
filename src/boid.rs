use ncollide3d::nalgebra::{Point3, Vector3, Vector4};
use ncollide3d::nalgebra::geometry::UnitQuaternion;
use rand::{thread_rng, Rng};
use rand_distr::{Distribution, UnitSphere};

// Scaling factors
const TIME_SCALE: f32 = 1.0;
const MAX_VEL: f32 = 4.0;
const MIN_VEL: f32 = 1.0;

/// boid struct is a point with a velocity
#[derive(Clone)]
pub struct Boid {
    pos: Point3<f32>,
    vel: Vector3<f32>,
}

impl Boid {
    /// create new boid
    pub fn new(pos: Vec<f32>, vel: Vec<f32>) -> Self {
        // TODO: use from_row_slice instead
        Boid {
            pos: Point3::from(Vector3::from_vec(pos)),
            vel: Vector3::from_vec(vel),
        }
    }

    /// return position as array
    pub fn pos_array(&self) -> [f32; 3] {
        self.pos.coords.into()
    }

    /// return rotation from y-axis towards velocity
    pub fn rot_array(&self) -> [f32; 4] {
        let rot: Vector4<f32> = *UnitQuaternion::rotation_between(&Vector3::y_axis(), &self.vel)
            .unwrap_or(UnitQuaternion::from_axis_angle(
                &Vector3::x_axis(),
                std::f32::consts::PI,
            ))
            .as_vector();
        rot.into()
    }

    /// return velocity as array
    pub fn vel_array(&self) -> [f32; 3] {
        self.vel.into()
    }

    pub fn frame_update(&mut self, delta_time: f32) {
        // update position
        self.pos += self.vel * delta_time * TIME_SCALE;
    }

    /// Returns of vector of boids generated randomly inside a given sphere
    ///
    /// # Arguments
    ///
    /// * `c` - centre of the sphere
    /// * `r` - radius of the sphere
    /// * `n` - number of boids to be spawned
    pub fn spawn_boids(c: &[f32; 3], r: f32, n: usize) -> Vec<Boid> {
        let centre: Vector3<f32> = Vector3::from_row_slice(c);
        let mut boids: Vec<Boid> = Vec::new();
        let mut rng = thread_rng();

        for _ in 0..n {
            // create position by random offset from centre within given radius
            let off_value = r * rng.gen_range(-1.0, 1.0);
            let coords: [f32; 3] = UnitSphere.sample(&mut rng);
            let offset: Vector3<f32> =
                Vector3::<f32>::from_row_slice(&coords);
            let pos: Point3<f32> = Point3::from(centre + offset * off_value);

            // create random velocity with magnitude between MIN_VEL and MAX_VEL
            let vel_value: f32 = if rng.gen_bool(0.5) {
                rng.gen_range(MIN_VEL, MAX_VEL)
            } else {
                rng.gen_range(-MAX_VEL, -MIN_VEL)
            };
            let coords: [f32; 3] = UnitSphere.sample(&mut rng);
            let vel: Vector3<f32> = Vector3::<f32>::from_row_slice(&coords) * vel_value;

            // add boid to Vector
            boids.push(Boid { pos, vel })
        }

        boids
    }
}
