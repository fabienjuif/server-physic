extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::collections::HashSet;
use std::rc::Rc;
use std::{thread, time};
use std::sync::mpsc::{self, Sender};

use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use ncollide2d::events::{ContactEvent};
use nphysics2d::object::{BodyHandle, Material, ColliderHandle};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics2d::algebra::{Inertia2, Velocity2};
use nphysics_testbed2d::Testbed;
use nphysics_testbed2d::{GraphicsManager, WorldOwner};

const COLLIDER_MARGIN: f32 = 0.01;

fn create_ground (world: &mut World<f32>) {
    let material = Material::new(1.0, 0.0);

    let ground_radius = 50.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radius - COLLIDER_MARGIN,
        ground_radius - COLLIDER_MARGIN,
    )));

    let positions = [
        Isometry2::new(-Vector2::y() * ground_radius, na::zero()),
        Isometry2::new(Vector2::y() * ground_radius * 2.0, na::zero()),
        Isometry2::new(Vector2::new(ground_radius * 1.5, ground_radius * 0.5), na::zero()),
        Isometry2::new(Vector2::new(-ground_radius * 1.5, ground_radius * 0.5), na::zero()),
    ];

    for position in positions.iter() {
        world.add_collider(
            COLLIDER_MARGIN,
            ground_shape.clone(),
            BodyHandle::ground(),
            *position,
            material.clone(),
        );
    }
}

fn create_balls (world: &mut World<f32>, num: usize) -> Vec<ColliderHandle> {
    let material = Material::new(1.0, 0.0);
    let rad = 1.5;

    // let geom = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(rad - COLLIDER_MARGIN, rad - COLLIDER_MARGIN)));
    let inertia = Inertia2::new(1.0, 0.0);
    let center_of_mass = geom.center_of_mass();

    let mut handlers = vec![];
    for i in 0..num {
        let x = (i as f32 -1.0) * 4.0;

        let pos = Isometry2::new(Vector2::new(x, 3.0), 0.0);
        let handle = world.add_rigid_body(pos, inertia, center_of_mass);

        handlers.push(world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            handle,
            Isometry2::identity(),
            material.clone(),
        ));
    }

    handlers
}

fn test<F: Fn(&mut WorldOwner, &mut GraphicsManager, f32) + 'static>(world: World<f32>, callback: F) {
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(0.0, -30.0), 15.0);
    testbed.hide_performance_counters();

    testbed.add_callback(callback);
    testbed.run();
}

#[derive(Debug)]
struct Ball {
    id: usize,
    position: Isometry2<f32>,
    velocity: Velocity2<f32>,
}

impl Ball {
    fn new(id: usize, position: Isometry2<f32>, velocity: Velocity2<f32>) -> Ball {
        Ball {
            id,
            position,
            velocity,
        }
    }
}

fn physics(txBalls: Sender<Vec<Ball>>, txMessages: Sender<String>) {
    let mut world = World::new();

    create_ground(&mut world);
    let mut balls_handler = create_balls(&mut world, 2);

    let mut balls = HashSet::new();
    for handler in balls_handler.clone() {
        balls.insert(handler.uid());
    }

    let body_collision_handler = balls_handler.last().unwrap();
    let body_handler = world.collider_body_handle(*body_collision_handler).unwrap();

    let body = world.rigid_body_mut(body_handler).unwrap();
    body.set_linear_velocity(Vector2::new(30.0, 30.0));

    println!("[physics] start the simulation.");
    let ten_millis = time::Duration::from_millis(1000 / 60);
    for _ in 0..300 {
        // TODO: make it real 60FPS in the main thread
        thread::sleep(ten_millis);
        world.step();

        let sync_balls = balls_handler.iter().map(|&handler| {
            let body_handler = world.collider_body_handle(handler).unwrap();
            let rigid_body = world.rigid_body_mut(body_handler).unwrap();

            Ball::new(handler.uid(), rigid_body.position().clone(), rigid_body.velocity().clone())
        });
        txBalls.send(sync_balls.collect());
        txMessages.send(String::from("balls"));
    }

    println!("[physics] end.");
    txMessages.send(String::from("end"));
}

fn main() {
    let (txMessages, rxMessages) = mpsc::channel();
    let (txBalls, rxBalls) = mpsc::channel();

    let handle = thread::spawn(move || physics(txBalls, txMessages));

    loop {
        if let Ok(message) = rxMessages.recv() {
            match message.as_str() {
                "balls" => {
                    let sync_balls = rxBalls.try_recv().unwrap();
                    println!("[main] balls! {:?}", sync_balls);
                },
                "end" => {
                    break;
                },
                message => {
                    panic!("Unknown message: {}", message);
                }
            }
        }
    }

    handle.join();


    // test(world, move |_,_,_| {
    //     let mut step = 1;

    //     |world_owner: World<f32>, _, _| {
    //         step += 1;
    //         let world = world_owner.get_mut();

    //         println!("{}", step);

    //         for contact in world.contact_events() {
    //             match contact {
    //                 ContactEvent::Started(handle_a, handle_b) => {
    //                     if balls.contains(&handle_a.uid())
    //                     && balls.contains(&handle_b.uid()) {
    //                         println!("BALL vs BALL!");
    //                     }
    //                 },
    //                 ContactEvent::Stopped(handle_a, handle_b) => {
    //                     if balls.contains(&handle_a.uid())
    //                     && balls.contains(&handle_b.uid()) {
    //                         // panic!("Done.");
    //                     }

    //                 },
    //             }
    //         }
    //     }
    // });
}
