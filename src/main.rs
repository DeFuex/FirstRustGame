extern crate ggez;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate rand;

use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::{self, EventHandler, Keycode, Mod};
use ggez::graphics::{circle, clear, line, present, DrawMode, Point2, Vector2 as gVector2};
use ggez::timer;
use ggez::{Context, ContextBuilder, GameResult};

use rand::Rng;

use na::{Translation2, UnitComplex, Vector1, Vector2};
use ncollide::shape::{Ball, Plane};
use nphysics2d::math::Orientation;
use nphysics2d::object::{RigidBody, RigidBodyHandle};
use nphysics2d::world::World;

const WINDOW_WIDTH: u32 = 800;
const WINDOW_HEIGHT: u32 = 600;

const PLAYER_LIFE: f32 = 1.0;
const PLAYER_BBOX: f32 = 2.0;
const PLAYER_R: f32 = 10.0;

const WALL_LIFE: f32 = 10.0;
const WALL_BBOX: f32 = 1.0;

const FLOOR_Y: f32 = 400.0;

const MAX_PHYSICS_VEL: f32 = 250.0;

// Acceleration in pixels per second.
const PLAYER_THRUST: f32 = 100.0;
// Rotation in radians per second.
const PLAYER_TURN_RATE: f32 = 3.0;

enum ActorType {
    Player,
    Wall,
}

struct Actor {
    tag: ActorType,
    body: RigidBody<f32>,
    pos: Point2,
    direction: f32,
    velocity: gVector2,
    ang_vel: f32,
    mov_left: bool,
    mov_right: bool,
    bbox_size: f32,
    life: f32,
}

struct Assets {
    player_image: ggez::graphics::Image,
    shot_image: ggez::graphics::Image,
    rock_image: ggez::graphics::Image,
    font: ggez::graphics::Font,
    shot_sound: ggez::audio::Source,
    hit_sound: ggez::audio::Source,
}

pub struct Game {
    world: World<f32>,
    //TODO: turn player type into Actor instead of directly using RigidBodyHandle<f32>
    player: Actor,
    player_body: RigidBodyHandle<f32>,
    // walls: Vec<Actor>,
    is_walls_setup: bool,
    walls: Vec<Actor>,
    wall_bodies: Vec<RigidBodyHandle<f32>>,
    wall_collision: bool,
    wall_pos_col_x: f32,
    wall_pos_col_y: f32,
    // level: i32,
    // score: i32,
    // assets: Assets,
    // cubes: Vec<Cube>,
    // timer: f64,
    // temp: f64,
    // ground_y: f64,
    // mov_left: bool,
    // mov_right: bool,
}

fn create_player() -> Actor {
    Actor {
        tag: ActorType::Player,
        body: RigidBody::new_dynamic(Ball::new(PLAYER_R), 1.0, 0.0, 0.0),
        pos: Point2::origin(),
        direction: 0.,
        velocity: na::zero(),
        ang_vel: 0.,
        mov_left: false,
        mov_right: false,
        bbox_size: PLAYER_BBOX,
        life: PLAYER_LIFE,
    }
}

fn create_wall() -> Actor {
    Actor {
        tag: ActorType::Wall,
        body: RigidBody::new_static(Plane::new(Vector2::new(0.0, 1.0)), 0.3, 0.6),
        pos: Point2::origin(),
        direction: 0.,
        velocity: na::zero(),
        ang_vel: 0.,
        mov_left: false,
        mov_right: false,
        bbox_size: WALL_BBOX,
        life: WALL_LIFE,
    }
}

fn create_walls(num: i32, blocked_pos: Point2) -> Vec<Actor> {
    let new_wall = |_| {
        let mut wall = create_wall();
        let wall_angle = rand::random::<f32>() * 2.0 * std::f32::consts::PI;
        // let wall_distance = rand:random::<f32>();
        wall.pos = blocked_pos + vec_from_angle(wall_angle) * 2.0;
        wall.velocity = random_vector(50.0);
        wall
    };
    (0..num).map(new_wall).collect()
}

/// Create a unit vector representing the
/// given angle (in radians)
fn vec_from_angle(angle: f32) -> ggez::graphics::Vector2 {
    let vx = angle.sin();
    let vy = angle.cos();
    ggez::graphics::Vector2::new(vx, vy)
}

/// Just makes a random `Vector2` with the given max magnitude.
fn random_vector(max_magnitude: f32) -> ggez::graphics::Vector2 {
    let angle = rand::random::<f32>() * 2.0 * std::f32::consts::PI;
    let mag = rand::random::<f32>() * max_magnitude;
    vec_from_angle(angle) * (mag)
}

// fn player_handle_input(actor: &mut Actor, input: &InputState, dt: f32) {
//     actor.facing += dt * PLAYER_TURN_RATE * input.xaxis;

//     if input.yaxis > 0.0 {
//         player_thrust(actor, dt);
//     }
// }

fn player_handle_input(actor: &mut Actor, dt: f32) {
    actor.direction += dt * PLAYER_TURN_RATE; //* input.xaxis;

    // if input.yaxis > 0.0 {
    player_thrust(actor, dt);
    // }
}

fn player_thrust(actor: &mut Actor, dt: f32) {
    let direction_vector = vec_from_angle(actor.direction);
    let thrust_vector = direction_vector * (PLAYER_THRUST);
    actor.velocity += thrust_vector * (dt);
}

fn update_actor_position(actor: &mut Actor, dt: f32) {
    // Clamp the velocity to the max efficiently
    let norm_sq = actor.velocity.norm_squared();
    if norm_sq > MAX_PHYSICS_VEL.powi(2) {
        actor.velocity = actor.velocity / norm_sq.sqrt() * MAX_PHYSICS_VEL;
    }
    // println!("SECONDS: {}", dt);
    // println!("VELOCITY: {}", actor.velocity);
    // let dv = ([[actor.velocity], [0, 1]].concat()) * (dt);
    let dv = actor.velocity * (dt);
    // println!("DIFF pos: {}", dv);
    actor.pos += dv;
    // println!("CHANGE player pos: {}", actor.pos);
    actor.direction += actor.ang_vel;
}

impl Game {
    fn new(_ctx: &mut Context) -> GameResult<Game> {
        print_instructions();

        let mut world = World::new();
        world.set_gravity(Vector2::new(0.0, 200.0));
        let mut floor = RigidBody::new_static(Plane::new(Vector2::new(0.0, -1.0)), 0.0, 0.0);
        floor.append_translation(&Translation2::new(0.0, FLOOR_Y));
        world.add_rigid_body(floor);

        let mut player = create_player();
        let walls = create_walls(5, player.pos);

        let wall_bodies = Vec::new();

        // let mut player = RigidBody::new_dynamic(Ball::new(PLAYER_R), 1.0, 0.0, 0.0);
        player.body.set_inv_mass(1.5);
        player
            .body
            .append_translation(&Translation2::new(400.0, 300.0));
        let player_body = world.add_rigid_body(player.body.clone());

        let s = Game {
            world,
            player,
            player_body,
            is_walls_setup: false,
            walls,
            wall_bodies,
            wall_collision: false,
            wall_pos_col_x: 0.0,
            wall_pos_col_y: 0.0,
        };
        Ok(s)
    }

    fn setup_walls(&mut self) {
        if !self.is_walls_setup {
            for wall in &mut self.walls {
                wall.body.set_inv_mass(0.);
                wall.body.append_rotation(&UnitComplex::new(100.0));
                // wall.body.append_rotation(&UnitComplex::new(rand::thread_rng().gen_range(0.0, 100.0)));
                wall.body.append_translation(&Translation2::new(
                    0.0,
                    rand::thread_rng().gen_range(200.0, 300.0),
                )); //rand::thread_rng().gen_range(0., 400.0)));
                self.wall_bodies
                    .push(self.world.add_rigid_body(wall.body.clone()));
            }
            self.is_walls_setup = true;
        }
    }

    fn refresh_horizontal_vel(&mut self) {
        let mut player: std::cell::RefMut<RigidBody<f32>> = self.player_body.borrow_mut();
        let current = player.lin_vel();
        let mut dir = 0.0;
        let mut rot = 0.0;
        if self.player.mov_left && !self.player.mov_right {
            dir = -1.0;
            rot = -800.0
        } else if self.player.mov_right && !self.player.mov_left {
            dir = 1.0;
            rot = 800.0
        }
        player.set_lin_vel(Vector2::new(dir * 400.0, current.y));
        player.set_rotation(UnitComplex::new(rot));
    }

    fn handle_collisions(&mut self) {
        // if player collides with wall, decide to stick to it with space/jump button
        for wall in &mut self.walls {
            let pdistance = wall.pos - self.player.pos;

            /* TEST VALUES */
            println!("wall pos: {}", wall.pos);
            println!("player pos: {}", self.player.pos);
            println!("{}", wall.pos - self.player.pos);
            println!("distance: {}", pdistance.norm());
            println!("player box: {}", self.player.bbox_size);
            println!("wall box: {}", wall.bbox_size);
            println!("box size: {}", self.player.bbox_size + wall.bbox_size);
            println!(
                "{}",
                (pdistance.norm() / 1000.0) < (self.player.bbox_size + wall.bbox_size)
            );
            /* TEST VALUES */

            if (pdistance.norm() / 100.) < (self.player.bbox_size + wall.bbox_size) {
                // self.player.life = 0.0;
                self.wall_collision = true;
                self.wall_pos_col_x = wall.pos.x;
                self.wall_pos_col_y = wall.pos.y;
                break;
            } else {
                self.wall_collision = false;
            }

            println!("WALL_COLLISION: {}", self.wall_collision);
        }
    }
}

impl EventHandler for Game {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        const DESIRED_FPS: u32 = 60;
        while timer::check_update_time(ctx, DESIRED_FPS) {
            let seconds = 1.0 / (DESIRED_FPS as f32);

            self.world.step(seconds);

            player_handle_input(&mut self.player, seconds);

            // Update the pos of all actors to avoid e.g.: self.player.pos = [0, 0]
            update_actor_position(&mut self.player, seconds);
        }

        self.setup_walls();

        self.handle_collisions();

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        clear(ctx);
        let pos = self.player_body.borrow().position_center();
        circle(
            ctx,
            DrawMode::Fill,
            Point2::new(pos.x, pos.y),
            PLAYER_R,
            0.1,
        )?;
        line(
            ctx,
            &[
                Point2::new(0.0, FLOOR_Y),
                Point2::new(WINDOW_WIDTH as f32, FLOOR_Y),
            ],
            1.0,
        )?;

        // Loop over all objects drawing them...
        {
            // for wall in &self.walls {
            //     let mut local = wall.borrow();
            // }

            for wall_body in &self.wall_bodies {
                // let mut coords = wall_body.borrow_mut();
                // let local = coords.local_to_world;
                line(
                    ctx,
                    &[
                        Point2::new(
                            wall_body.borrow().position_center().x,
                            wall_body.borrow().position_center().y,
                        ),
                        Point2::new(WINDOW_WIDTH as f32, -100.0),
                    ],
                    1.0,
                )?; //wall_body.borrow().position_center().coords.data
            }
        }

        present(ctx);
        Ok(())
    }

    fn key_down_event(&mut self, _ctx: &mut Context, keycode: Keycode, _keymod: Mod, repeat: bool) {
        if !repeat {
            match keycode {
                Keycode::Left => {
                    self.player.mov_left = true;
                }
                Keycode::Right => {
                    self.player.mov_right = true;
                }
                Keycode::Space => {
                    if self.wall_collision {
                        self.world.set_gravity(Vector2::new(0.0, -200.0));
                    } else {
                        self.world.set_gravity(Vector2::new(0.0, 200.0));
                    }

                    let mut player: std::cell::RefMut<RigidBody<f32>> =
                        self.player_body.borrow_mut();
                    if player.position_center().y + PLAYER_R > FLOOR_Y - 0.1 {
                        player.apply_central_impulse(Vector2::new(0.0, -150.0));
                    }
                }
                _ => (),
            }
            self.refresh_horizontal_vel();
        }
    }
    fn key_up_event(&mut self, _ctx: &mut Context, keycode: Keycode, _keymod: Mod, repeat: bool) {
        if !repeat {
            match keycode {
                Keycode::Left => {
                    self.player.mov_left = false;
                }
                Keycode::Right => {
                    self.player.mov_right = false;
                }
                Keycode::Space => {
                    if self.wall_collision {
                        // if (self.wall_pos_col_y > self.player.pos.y) &&
                        // (self.wall_pos_col_x > self.player.pos.x) {
                        self.world.set_gravity(Vector2::new(0.0, 200.0));
                        // }
                    }
                    // else {
                    //     self.world.set_gravity(Vector2::new(0.0, -200.0));
                    // }
                }
                _ => (),
            }
            self.refresh_horizontal_vel();
        }
    }
}

pub fn main() {
    let cb = ContextBuilder::new("rust-game-experiment", "ggez")
        .window_setup(WindowSetup::default().title("My first Rust game"))
        .window_mode(WindowMode::default().dimensions(WINDOW_WIDTH, WINDOW_HEIGHT));
    let ctx = &mut cb.build().unwrap();
    println!();
    println!();
    let state = &mut Game::new(ctx).unwrap();
    event::run(ctx, state).unwrap();
}

fn print_instructions() {
    println!();
    println!("Welcome to Ball!");
    println!();
    println!("How to play:");
    println!("Left/Right arrow keys move your ball, space bar lets the ball jump");
    println!();
}

// fn draw_actor_with_assets(
//     assets: &mut Assets,
//     ctx: &mut Context,
//     actor: &mut Actor,
//     world_coords: (u32, u32),
// ) -> GameResult<()> {
//     let (screen_w, screen_h) = world_coords;
//     let pos = world_to_screen_coords(screen_w, screen_h, actor.pos);
//     let image = assets.actor_image(actor);
//     let drawparams = ggez::graphics::DrawParam {
//         dest: pos,
//         rotation: actor.direction as f32, //facing var for rotation
//         offset: ggez::graphics::Point2::new(0.5, 0.5),
//         ..Default::default()
//     };
//     ggez::graphics::draw_ex(ctx, image, drawparams)
// }

/// Translates the world coordinate system, which
/// has Y pointing up and the origin at the center,
/// to the screen coordinate system, which has Y
/// pointing downward and the origin at the top-left,
fn world_to_screen_coords(screen_width: u32, screen_height: u32, point: Point2) -> Point2 {
    let width = screen_width as f32;
    let height = screen_height as f32;
    let x = point.x + width / 2.0;
    let y = height - (point.y + height / 2.0);
    Point2::new(x, y)
}
