//! This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of 
//! multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.
//! 
//! Example usage:
//! ```ignore
//! fn main() {
//!     rosrust::init("listener");
//!     let listener = TfListener::new();
//!     
//!     let rate = rosrust::rate(1.0);
//!     while rosrust::is_ok() {
//!         let tf = listener.lookup_transform("camera", "base_link", ros::Time::now());
//!         println!("{:?}", tf);
//!         rate.sleep();
//!     }
//! }
//!``` 
mod core;
mod buffer;
mod transforms;
mod graph;
mod utils;
mod chain;
mod listener;



