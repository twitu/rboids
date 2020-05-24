use ncollide3d::nalgebra::{Point3, Vector3, Vector4};
use ncollide3d::nalgebra::geometry::UnitQuaternion;
use ncollide3d::query::{RayCast, Ray};
use ncollide3d::math::Isometry;
use rand::{thread_rng, Rng};
use rand_distr::{Distribution, UnitSphere};
use ncollide3d::nalgebra::clamp;

// Scaling factors
const TIME_SCALE: f32 = 1.0;
const MAX_VEL: f32 = 4.0;
const MIN_VEL: f32 = 1.0;
const RAY_NUMS: usize = 100;
const OBSTACLE_DIST: f32 = 5.0;

// rule weightage
const MAX_ACC: f32 = 3.0;
const MIN_ACC: f32 = 0.0;
const OBSTACLE_W: f32 = 6.0;

use lazy_static::lazy_static;

lazy_static! {
    static ref RAY_DIRS: [Vector3<f32>; RAY_NUMS] = {
        let mut ray_dirs = [Vector3::new(0.0, 0.0, 0.0); RAY_NUMS];
        // initialize ray angles
        let golden_ratio: f32 = 1.618;
        let angle_increment = 3.1415 * 2.0 * golden_ratio;

        for i in 0..RAY_NUMS {
            let t: f32 = i as f32 / RAY_NUMS as f32;
            let inclination: f32 = (1.0 - 2.0 * t).acos();
            let azimuth: f32 = angle_increment * i as f32;

            let x = inclination.sin() * azimuth.cos();
            let y = inclination.sin() * azimuth.sin();
            let z = inclination.cos();
            ray_dirs[i] = Vector3::new(x, y, z);
        }

        ray_dirs
    };
}

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

    /// return unobstructed direction closest to current velocity
    fn unobstructed_dir(&self, obs: &Vec<(Box<dyn RayCast<f32>>, Isometry<f32>)>) -> Option<Vector3<f32>> {
        // create a rotation to orient ray directions along velocity
        let ray_axis: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
        let rot = UnitQuaternion::rotation_between(&ray_axis, &self.vel).unwrap_or(
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::PI),
        );

        // iterate over all ray directions
        // create a ray from each direction and check for collision
        let mut best_dir: Option<Vector3<f32>> = None;
        for dir in RAY_DIRS.iter() {
            let ray = Ray {
                origin: self.pos,
                dir: rot * dir,
            };

            // if direction is unobstructed store it
            // after correcting it's orientation
            if !collided(obs, ray) {
                best_dir = Some(rot * dir);
                break;
            }
        }

        // It is possible that there is no unobstructed direction
        // but that is highly unlikely and possible only when the boid
        // is entirely surrounded by obstacles.
        best_dir
    }

    /// calculate clamped acceleration in the direction of `vel`
    ///
    /// `vel` - vel should be a unit vector to ensure correct calculations
    fn calc_acc(&self, vel: &Vector3<f32>) -> Vector3<f32> {
        let mut acc = vel * MAX_VEL - self.vel;
        acc.set_magnitude(clamp(acc.magnitude(), MIN_ACC, MAX_ACC));
        acc
    }

    /// apply rules to calculate acceleration
    fn apply_rules(&self, obs: &Vec<(Box<dyn RayCast<f32>>, Isometry<f32>)>) -> Vector3<f32> {
        let mut acc: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);

        // check if current heading is obstructed
        let cur_ray: Ray<f32> = Ray{origin: self.pos, dir: self.vel.normalize()};
        if collided(obs, cur_ray) {
            // try to find an unobstructed direction
            // only affect acceleration if unobstructed direction exists
            if let Some(dir) = self.unobstructed_dir(obs) {
                acc += self.calc_acc(&dir) * OBSTACLE_W;
            }
        }

        acc
    }

    pub fn frame_update(&mut self, obs: &Vec<(Box<dyn RayCast<f32>>, Isometry<f32>)>, delta_time: f32) {
        // update position
        self.pos += self.vel * delta_time * TIME_SCALE;

        // update velocity
        let mut new_vel = self.vel + self.apply_rules(obs);
        new_vel.set_magnitude(clamp(new_vel.magnitude(), MIN_VEL, MAX_VEL));
        self.vel = new_vel;
    }
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

/// check if a ray collides with the given obstacles
fn collided(obs: &Vec<(Box<dyn RayCast<f32>>, Isometry<f32>)>, ray: Ray<f32>) -> bool {
    obs.iter()
        .any(|(shape, iso)| shape.intersects_ray(iso, &ray, OBSTACLE_DIST))
}
