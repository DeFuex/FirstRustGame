extern crate ggez;
extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate rand;

use ggez::{ContextBuilder, Context, GameResult};
use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::{ self, EventHandler, Keycode, Mod} ;
use ggez::graphics::{ clear, circle, line, present, DrawMode, Point2 };
use ggez::timer;

use na::{ Vector2, Isometry2 };
use ncollide2d::shape::{ Ball, Cuboid, ShapeHandle };
use ncollide2d::world::{ CollisionObjectHandle };
use nphysics2d::world::World;
use nphysics2d::object::{ Material, BodyHandle };
use nphysics2d::volumetric::Volumetric;

const WINDOW_WIDTH: u32 = 800;
const WINDOW_HEIGHT: u32 = 600;

const PLAYER_LIFE: f32 = 1.0;
const PLAYER_BBOX: f32 = 2.0;
const PLAYER_R: f32 = 10.0;

const WALL_LIFE: f32 = 10.0;
const WALL_BBOX: f32 = 1.0;

const FLOOR_LIFE: f32 = 1.0;
const FLOOR_BBOX: f32 = 1.0;
const FLOOR_Y: f32 = 400.0;

const MAX_PHYSICS_VEL: f32 = 250.0;

// Acceleration in pixels per second.
const PLAYER_THRUST: f32 = 100.0;
// Rotation in radians per second.
const PLAYER_TURN_RATE: f32 = 3.0;

const COLLIDER_MARGIN: f32 = 0.01;

enum ActorType {
    Player,
    Floor,
    Wall,
}

struct Actor {
    tag: ActorType,
    collider_handle: CollisionObjectHandle,
    pos: Point2,
    direction: f32,
    velocity: Vector2<f32>,
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
    player: Actor,
    floor: Actor,
    is_walls_setup: bool,
    walls: Vec<Actor>,
    wall_collision: bool,
    wall_pos_col_x: f32,
    wall_pos_col_y: f32,
}

/*
 * A floor that will collide with everything (default behaviour).
 */
fn create_floor(world: &mut World<f32>) -> Actor {
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let mut vecc = Vector2::new(0., 400.0);
    println!("{}", vecc);
    let ground_pos = Isometry2::new(vecc, 0.);
    let collider_handle = world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    Actor {
        tag: ActorType::Floor,
        collider_handle,
        pos: Point2::origin(),
        direction: 0.,
        velocity: na::zero(),
        ang_vel: 0.,
        mov_left: false,
        mov_right: false,
        bbox_size: FLOOR_BBOX,
        life: FLOOR_LIFE,
    }
}

fn create_player(world: &mut World<f32>) -> Actor {
    let num = (2000.0f32.sqrt()) as usize;
    let rad = 0.1;
    let shift = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    let x = 500. * rad - centerx;
    let y = 2.5 * rad + centery * 2.0 + 0.5;

    /*
    * Create the rigid body.
    */
    let pos = Isometry2::new(Vector2::new(x, y), na::zero());
    let handle = world.add_rigid_body(pos, inertia, center_of_mass);

    /*
        * Create the collider.
        */
    let collider_handle = world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry2::identity(),
        Material::default(),
    );

    // let geom = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    // let inertia = geom.inertia(1.0);
    // let center_of_mass = geom.center_of_mass();

    // let geom = ShapeHandle::new(Ball::new(PLAYER_R - COLLIDER_MARGIN));
    // let inertia = geom.inertia(1.0);
    // let center_of_mass = geom.center_of_mass();
    // let pos = Isometry2::new(Vector2::new(200., 0.), 0.0);
    // let handle = world.add_rigid_body(pos, inertia, center_of_mass);
    // let collider_handle = world.add_collider(
    //     COLLIDER_MARGIN,
    //     geom.clone(),
    //     handle,
    //     Isometry2::identity(),
    //     Material::default(),
    // );
    Actor {
        tag: ActorType::Player,
        collider_handle,
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

fn create_wall(world: &mut World<f32>) -> Actor {
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(
        0.3 - COLLIDER_MARGIN,
        0.6 - COLLIDER_MARGIN,
    )));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();
    let pos = Isometry2::new(Vector2::new(0.0, 1.0), 0.0);
    let handle = world.add_rigid_body(pos, inertia, center_of_mass);
    let collider_handle = world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry2::identity(),
        Material::default(),
    );
    Actor {
        tag: ActorType::Wall,
        collider_handle,
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

fn create_walls(world: &mut World<f32>, num: i32, blocked_pos: Point2) -> Vec<Actor> {
    // let new_wall = |_| {
    //     let mut wall = create_wall(&mut world);
    //     // let wall_angle = rand::random::<f32>() * 2.0 * std::f32::consts::PI;
    //     // let wall_distance = rand:random::<f32>();
    //     // wall.pos = blocked_pos + vec_from_angle(wall_angle) * 2.0;
    //     // wall.velocity = random_vector(50.0);
    //     wall
    // };
    let mut walls : Vec<Actor> = Vec::new();
    for x in 0..num {
        walls.push(create_wall(world));
    }
    walls
    // (0..num).map(new_wall).collect()
}

/// Create a unit vector representing the
/// given angle (in radians)
fn vec_from_angle(angle: f32) -> Vector2<f32> {
    let vx = angle.sin();
    let vy = angle.cos();
    Vector2::new(vx, vy)
}

/// Just makes a random `Vector2` with the given max magnitude.
fn random_vector(max_magnitude: f32) -> Vector2<f32> {
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
    // TODO re-implement the pos change
    // actor.pos += dv;
    // println!("CHANGE player pos: {}", actor.pos);
    actor.direction += actor.ang_vel;
}

impl Game {
    fn new(_ctx: &mut Context) -> GameResult<Game> {

        print_instructions();

        let mut world = World::new();
        world.set_gravity(Vector2::new(0.0, 100.));

        let mut floor = create_floor(&mut world);
        let mut player = create_player(&mut world);
        let walls = create_walls(&mut world, 5, player.pos);

        let s = Game {
                world,
                floor,
                player, 
                is_walls_setup: false,
                walls, 
                wall_collision: false,
                wall_pos_col_x: 0.0,
                wall_pos_col_y: 0.0,
            };
        Ok(s)
    }

    // fn refresh_horizontal_vel(&mut self) {
    //     let mut player: std::cell::RefMut<RigidBody<f32>> = self.world.rigid_body(self.player_body);
    //     let current = player.lin_vel();
    //     let mut dir = 0.0;
    //     let mut rot = 0.0;
    //     if self.player.mov_left && !self.player.mov_right {
    //         dir = -1.0;
    //         rot = -800.0
    //     } else if self.player.mov_right && !self.player.mov_left {
    //         dir = 1.0;
    //         rot = 800.0
    //     }
    //     player.set_lin_vel(Vector2::new(dir * 400.0, current.y));
    //     player.set_rotation(UnitComplex::new(rot));
    // }

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
            println!("{}", (pdistance.norm()/1000.0) < (self.player.bbox_size + wall.bbox_size));
            /* TEST VALUES */

            if (pdistance.norm()/100.) < (self.player.bbox_size + wall.bbox_size) {
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

            self.world.step();

            player_handle_input(&mut self.player, seconds);

            // Update the pos of all actors to avoid e.g.: self.player.pos = [0, 0]
            update_actor_position(&mut self.player, seconds);
        }

        self.handle_collisions();

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        clear(ctx);
        let player_collider = self.world.collider(self.player.collider_handle).expect("Player Collider not found.");
        let player_pos = player_collider.position().translation.vector;
        println!("drawn player pos.x: {}", player_pos.x);
        println!("drawn player pos.y: {}", player_pos.y);
        circle(ctx, DrawMode::Fill, Point2::new(player_pos.x, player_pos.y), PLAYER_R, 0.1)?;
        
        let floor_collider = self.world.collider(self.floor.collider_handle).expect("Floor Collider not found");
        let floor_pos = floor_collider.position().translation.vector;
        println!("drawn floor pos.x: {}", floor_pos.x);
        println!("drawn floor pos.y: {}", floor_pos.y);
        line(ctx, &[Point2::new(0.0, floor_pos.y), Point2::new(WINDOW_WIDTH as f32, floor_pos.y)], 1.0)?;

        // // Loop over all objects drawing them...
        // {
        //     // for wall in &self.walls {
        //     //     let mut local = wall.borrow();
        //     // }

        //     for wall_body in &self.wall_bodies {
        //         // let mut coords = wall_body.borrow_mut();
        //         // let local = coords.local_to_world;
        //         line(ctx, &[Point2::new(
        //             wall_body.borrow().position_center().x, 
        //             wall_body.borrow().position_center().y), 
        //             Point2::new(WINDOW_WIDTH as f32, 
        //             -100.0)], 1.0)?; //wall_body.borrow().position_center().coords.data
        //     }
        // }

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

                    // TODO bring back jumping
                    // let mut player: std::cell::RefMut<RigidBody<f32>> = self.player_body.borrow_mut();
                    // if player.position_center().y + PLAYER_R > FLOOR_Y - 0.1 {
                    //     player.apply_central_impulse(Vector2::new(0.0, -150.0));
                    // }
                }
                _ => ()
            }
            // self.refresh_horizontal_vel();
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
                _ => ()
            }
            // self.refresh_horizontal_vel();
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