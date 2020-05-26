use rayon::prelude::*;
use three;
use three::{Mesh, Object};
use ncollide3d::nalgebra::geometry::UnitQuaternion;
use ncollide3d::nalgebra::{Vector3, Vector4};

mod boid;
use boid::Boid;

const BACKGROUND_C: u32 = 0xFDEBD0; // peach
const GROUND_C: u32 = 0x2ECC71; // grass green
const TREE_C: u32 = 0xCD5C5C; // Indian red
const SUN_C: u32 = 0xF4D03F; // muddy yellow
const ORIGIN: [f32; 3] = [0.0, 0.0, 0.0];
const SPAWN_CENTRE: [f32; 3] = [0.0, 0.0, 0.0];
const SPAWN_RADIUS: f32 = 6.0;
const SPAWN_NUMBER: usize = 500;
const SPAWN_COLOURS: [u32; 3] = [ 0xFFA500 , 0x8E44AD, 0x1C2833 ];
const FPS: u32 = 60;
const DELTA: f32 = 1.0 / FPS as f32;
const CAMERA_ROTATION: f32 = DELTA * std::f32::consts::PI / 20.0;  // 18 degrees per second

fn main() {
    // add window
    let mut win = three::Window::new("rboids - not a flock more");
    win.scene.background = three::Background::Color(BACKGROUND_C);

    // add camera
    let cam = win.factory.perspective_camera(60.0, 1.0..1000.0);
    let mut cam_pos: [f32; 3] = [50.0, 10.0, 50.0];
    let cam_rot_cos = CAMERA_ROTATION.cos();
    let cam_rot_sin = CAMERA_ROTATION.sin();

    // add lighting to scene
    add_lighting_to_scene(&mut win);

    // add objects to scene
    add_objects_to_scene(&mut win);

    // create obstacles
    let obstacles = boid::create_obstacles();

    // create boid
    let mut boids: Vec<Boid> = boid::spawn_boids(&SPAWN_CENTRE, SPAWN_RADIUS, SPAWN_NUMBER);
    let cones: Vec<Mesh> = spawn_cones(&mut win);

    // text for fps
    let font = win.factory.load_font_karla();
    let mut fps_counter = win.factory.ui_text(&font, "FPS: 00");

    // render scene
    while win.update() && !win.input.hit(three::KEY_ESCAPE) {
        // update camera transform
        cam_pos[0] = cam_pos[0] * cam_rot_cos + cam_pos[2] * cam_rot_sin;
        cam_pos[2] = -cam_pos[0] * cam_rot_sin + cam_pos[2] * cam_rot_cos;
        cam.look_at(cam_pos, ORIGIN, Some([0.0f32, 1.0, 0.0].into()));

        // copy boid information
        let copy = boids.clone();
        let delta_time = win.input.delta_time();

        // maintain frame rate
        if delta_time < DELTA {
            std::thread::sleep(std::time::Duration::from_secs_f32(DELTA - delta_time));
        }

        // compute new boxy velocity and set it
        boids
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, b): (usize, &mut Boid)| {
                b.frame_update(i, &copy, &obstacles, DELTA)
            });
        boids
            .iter()
            .zip(cones.iter())
            .for_each(|(b, c)| c.set_transform(b.pos_array(), b.rot_array(), 1.0));

        // set fps
        fps_counter.set_text(format!("FPS: {}", 1.0 / delta_time));

        // render scene
        win.render(&cam);
    }
}

fn spawn_cones(win: &mut three::Window) -> Vec<Mesh> {
    let mut cones: Vec<Mesh> = Vec::new();
    let colours = SPAWN_COLOURS.len();
    for i in 0..SPAWN_NUMBER {
        let cone = {
            let geometry = three::Geometry::cylinder(0.0, 1.0, 1.5, 12);
            let material = three::material::Phong {
                color: SPAWN_COLOURS[i % colours],
                glossiness: 30.0,
            };
            win.factory.mesh(geometry, material)
        };
        win.scene.add(&cone);

        cones.push(cone);
    }

    cones
}

fn add_lighting_to_scene(win: &mut three::Window) {
    let hemisphere_light = win.factory.hemisphere_light(BACKGROUND_C, SUN_C, 0.3);
    win.scene.add(&hemisphere_light);
}

fn add_objects_to_scene(win: &mut three::Window) {

    // glass box
    let object = {
        let geometry = three::Geometry::cuboid(60.0, 60.0, 60.0);
        let material = three::material::Wireframe {
            color: three::color::GREEN,
        };
        win.factory.mesh(geometry, material)
    };
    win.scene.add(&object);

    // grassy ground
    let object = {
        let geometry = three::Geometry::plane(60.0, 60.0);
        let material = three::material::Lambert {
            color: GROUND_C,
            flat: false,
        };
        win.factory.mesh(geometry, material)
    };
    // set rotation -90 degrees about x-axis
    let rot = *UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_2).as_vector();
    let rot: [f32; 4] = rot.into();
    object.set_transform(
        [0.0, -30.0, 0.0],
        rot,
        1.0,
    );
    win.scene.add(&object);

    // tree trunks
    for (x, y, z) in [
        (20.0f32, -5.0, 20.0),
        (20.0, -5.0, -20.0),
        (-20.0, -5.0, -20.0),
        (-20.0, -5.0, 20.0),
    ]
    .iter()
    {
        let object = {
            let geometry = three::Geometry::cylinder(4.0, 4.0, 50.0, 12);
            let material = three::material::Lambert {
                color: TREE_C,
                flat: false,
            };
            win.factory.mesh(geometry, material)
        };
        object.set_position([*x, *y, *z]);
        win.scene.add(&object);
    }

    // add sun
    let object = {
        let geometry = three::Geometry::uv_sphere(6.0, 12, 12);
        let material = three::material::Lambert {
            color: SUN_C,
            flat: false,
        };
        win.factory.mesh(geometry, material)
    };
    object.set_position([0.0, 15.0, 0.0]);
    win.scene.add(&object);
}

pub fn rotation_about_y_axis(pos: &[f32; 3], ang: f32) -> [f32; 3] {
    let v = Vector3::from_row_slice(pos);
    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), ang);
    (rot * v).into()
}
