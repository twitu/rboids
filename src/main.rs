use three;
use three::Object;

const BACKGROUND_C: u32 = 0xF0E0B6;

fn main() {
    // add window
    let mut win = three::Window::new("rboids - not a flock more");
    win.scene.background = three::Background::Color(BACKGROUND_C);

    // add camera
    let cam = win.factory.perspective_camera(60.0, 1.0..1000.0);
    cam.look_at([5.0, 5.0, 5.0], [0.0, 0.0, 0.0], None);
    win.scene.add(&cam);

    // add origin
    let origin = {
        let geometry = three::Geometry::uv_sphere(1.0, 12, 12);
        let material = three::material::Wireframe { color: three::color::GREEN };
        win.factory.mesh(geometry, material)
    };
    origin.set_position([0.0, 0.0, 0.0]);
    win.scene.add(&origin);

    // start scene
    while win.update() && !win.input.hit(three::KEY_ESCAPE) {
        win.render(&cam);
    }
}
