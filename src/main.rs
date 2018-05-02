extern crate ggez;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;

use ggez::{ContextBuilder, Context, GameResult};
use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::{ self, EventHandler, Keycode, Mod} ;
use ggez::graphics::{ clear, circle, line, present, DrawMode, Point2 };
use ggez::timer;

use na::{Vector2, Translation2, UnitComplex};
use ncollide::shape::{Ball, Plane};
use nphysics2d::world::World;
use nphysics2d::object::{RigidBody, RigidBodyHandle};

const WINDOW_WIDTH: u32 = 800;
const WINDOW_HEIGHT: u32 = 600;

const PLAYER_LIFE: f32 = 1.0;
const PLAYER_BBOX: f32 = 12.0;
const PLAYER_R: f32 = 10.0;

const WALL_LIFE: f32 = 10.0;
const WALL_BBOX: f32 = 5.0;

const FLOOR_Y: f32 = 400.0;

enum ActorType {
    Player,
    Wall,
}

struct Actor {
    tag: ActorType,
    body: RigidBody<f32>,
    pos: Point2,
    direction: f32,
    velocity: Vector2<f32>,
    mov_left: bool,
    mov_right: bool,
    bbox_size: f32,
    life: f32,
}

pub struct Game {
    world: World<f32>,
    //TODO: turn player type into Actor instead of directly using RigidBodyHandle<f32>
    player: Actor,
    player_body: RigidBodyHandle<f32>,
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
        mov_left: false,
        mov_right: false,
        bbox_size: PLAYER_BBOX,
        life: PLAYER_LIFE,
    }
}

fn create_wall() -> Actor {
    Actor {
        tag: ActorType::Wall,
        body: RigidBody::new_static(Plane::new(Vector2::new(0.0, -1.0)), 0.0, 0.0),
        pos: Point2::origin(),
        direction: 0.,
        velocity: na::zero(),
        mov_left: false,
        mov_right: false,
        bbox_size: WALL_BBOX,
        life: WALL_LIFE,
    }
}

impl Game {
    fn new(_ctx: &mut Context) -> GameResult<Game> {
        let mut world = World::new();
        world.set_gravity(Vector2::new(0.0, 200.0));
        let mut floor = RigidBody::new_static(Plane::new(Vector2::new(0.0, -1.0)), 0.0, 0.0);
        floor.append_translation(&Translation2::new(0.0, FLOOR_Y));
        world.add_rigid_body(floor);

        let mut player = create_player();

        // let mut player = RigidBody::new_dynamic(Ball::new(PLAYER_R), 1.0, 0.0, 0.0);
        player.body.set_inv_mass(0.9);
        player.body.append_translation(&Translation2::new(300.0, 200.0));
        let player_body = world.add_rigid_body(player.body.clone());
        // world.
        let s = Game { world, player, player_body };
        Ok(s)
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
}

impl EventHandler for Game {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        const DESIRED_FPS: u32 = 60;
        while timer::check_update_time(ctx, DESIRED_FPS) {
            self.world.step(1.0 / DESIRED_FPS as f32);
        }
        Ok(())
    }
    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        clear(ctx);
        let pos = self.player_body.borrow().position_center();
        circle(ctx, DrawMode::Fill, Point2::new(pos.x, pos.y), PLAYER_R, 0.1)?;
        line(ctx, &[Point2::new(0.0, FLOOR_Y), Point2::new(WINDOW_WIDTH as f32, FLOOR_Y)], 1.0)?;
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
                    let mut player: std::cell::RefMut<RigidBody<f32>> = self.player_body.borrow_mut();
                    if player.position_center().y + PLAYER_R > FLOOR_Y - 0.1 {
                        player.apply_central_impulse(Vector2::new(0.0, -150.0));
                    }
                }
                _ => ()
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
                _ => ()
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