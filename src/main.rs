use three;
use three::{Object, Mesh};
use ncollide3d::query::RayCast;
use ncollide3d::math::Isometry;
use ncollide3d::shape::{Plane, Cylinder};
use ncollide3d::nalgebra::Vector3;

mod boid;
use boid::Boid;

const BACKGROUND_C: u32 = 0xF0E0B6;
const SPAWN_CENTRE: [f32; 3] = [0.0, 0.0, 0.0];
const SPAWN_RADIUS: f32 = 3.0;
const SPAWN_NUMBER: usize = 10;

fn main() {
    // add window
    let mut win = three::Window::new("rboids - not a flock more");
    win.scene.background = three::Background::Color(BACKGROUND_C);

    // add camera
    let cam = win.factory.perspective_camera(60.0, 1.0..1000.0);
    let mut controls = three::controls::Orbit::builder(&cam)
        .position([10.0, 10.0, 10.0])
        .up([0.0, 1.0, 0.0])
        .build();

    // add objects to scene
    add_objects_to_scene(&mut win);

    // create obstacles
    let obstacles = create_obstacles();

    // create boid
    let mut boids: Vec<Boid> = boid::spawn_boids(&SPAWN_CENTRE, SPAWN_RADIUS, SPAWN_NUMBER);
    let cones: Vec<Mesh> = spawn_cones(&mut win);

    // render scene
    while win.update() && !win.input.hit(three::KEY_ESCAPE) {
        // update camera transform
        controls.update(&win.input);

        // compute new boxy velocity and set it
        boids.iter_mut().for_each(|b: &mut Boid| b.frame_update(&obstacles, win.input.delta_time()));
        boids.iter().zip(cones.iter()).for_each(|(b, c)| c.set_transform(b.pos_array(), b.rot_array(), 1.0));

        // render scene
        win.render(&cam);
    }
}

fn spawn_cones(win: &mut three::Window) -> Vec<Mesh> {
    let mut cones: Vec<Mesh> = Vec::new();
    for _ in 0..SPAWN_NUMBER {
        let cone = {
            let geometry = three::Geometry::cylinder(0.0, 1.0, 1.5, 12);
            let material = three::material::Wireframe { color: three::color::BLACK };
            win.factory.mesh(geometry, material)
        };
        win.scene.add(&cone);

        cones.push(cone);
    }

    cones
}

fn add_objects_to_scene(win: &mut three::Window) {
    // add axes
    let x_edge = {
        let geometry = three::Geometry::uv_sphere(0.2, 12, 12);
        let material = three::material::Wireframe { color: three::color::MAGENTA };
        win.factory.mesh(geometry, material)
    };
    x_edge.set_position([5.0, 0.0, 0.0]);
    win.scene.add(&x_edge);

    let y_edge = {
        let geometry = three::Geometry::uv_sphere(0.2, 12, 12);
        let material = three::material::Wireframe { color: three::color::BLUE };
        win.factory.mesh(geometry, material)
    };
    y_edge.set_position([0.0, 5.0, 0.0]);
    win.scene.add(&y_edge);

    let z_edge = {
        let geometry = three::Geometry::uv_sphere(0.2, 12, 12);
        let material = three::material::Wireframe { color: three::color::YELLOW };
        win.factory.mesh(geometry, material)
    };
    z_edge.set_position([0.0, 0.0, 5.0]);
    win.scene.add(&z_edge);

    let mbox = {
        let geometry = three::Geometry::cuboid(30.0, 30.0, 30.0);
        let material = three::material::Wireframe { color: three::color::GREEN };
        win.factory.mesh(geometry, material)
    };
    mbox.set_position([0.0, 0.0, 0.0]);
    win.scene.add(&mbox);

    let mcone = {
        let geometry = three::Geometry::cylinder(3.0, 3.0, 30.0, 12);
        let material = three::material::Wireframe { color: three::color::GREEN };
        win.factory.mesh(geometry, material)
    };
    mcone.set_position([-10.0, 0.0, 0.0]);
    win.scene.add(&mcone);
}

/// create vector of obstacles and there corresponding isometric transformations
/// The obstacle shape is contained in Box to allow for dynamic dispatch
fn create_obstacles() -> Vec<(Box<dyn RayCast<f32>>, Isometry<f32>)> {
    // create obstacles
    let mut obstacles: Vec<(Box<dyn RayCast<f32>>, Isometry<f32>)> = Vec::new();
    obstacles.push((
        Box::new(Plane::new(Vector3::x_axis())),
        Isometry::translation(-15.0, 0.0, 0.0)
    ));
    obstacles.push((
        Box::new(Plane::new(-Vector3::x_axis())),
        Isometry::translation(15.0, 0.0, 0.0)
    ));
    obstacles.push((
        Box::new(Plane::new(Vector3::y_axis())),
        Isometry::translation(0.0, -15.0, 0.0)
    ));
    obstacles.push((
        Box::new(Plane::new(-Vector3::y_axis())),
        Isometry::translation(0.0, 15.0, 0.0)
    ));
    obstacles.push((
        Box::new(Plane::new(Vector3::z_axis())),
        Isometry::translation(0.0, 0.0, -15.0)
    ));
    obstacles.push((
        Box::new(Plane::new(-Vector3::z_axis())),
        Isometry::translation(0.0, 0.0, 15.0)
    ));
    obstacles.push((
        Box::new(Cylinder::new(25.0, 3.0)),
        Isometry::translation(-10.0, 0.0, 0.0)
    ));

    obstacles
}
