use rosrust_msg as msgs;

use std::cmp::Ordering;


pub mod ros_msg {
    pub use super::msgs::{
        geometry_msgs::TransformStamped,
        geometry_msgs::Transform,
        geometry_msgs::Vector3,
        geometry_msgs::Quaternion,
        tf2_msgs::TFMessage,
        std_msgs::Header
    };
}


pub trait MessageConverter<Output=Self> {
    type MessageType;

    fn from_msg(msg: Self::MessageType) -> Output;
    fn toMsg(output: Output) -> Self::MessageType;
}

/**
 * NewType pattern on geometry_msgs::TransformStamped 
 */
#[derive(Debug, Clone)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform
}

impl MessageConverter for TransformStamped {
    type MessageType = ros_msg::TransformStamped;

    fn from_msg(msg: Self::MessageType) -> TransformStamped { todo!() }

    fn toMsg(output: TransformStamped) -> Self::MessageType { todo!() }
}

impl PartialEq for TransformStamped {
    fn eq(&self, other: &Self) -> bool {
        todo!();
    }
}

impl Eq for TransformStamped {}

impl Ord for  TransformStamped {
    fn cmp(&self, other: &TransformStamped) -> Ordering {
        self.header.stamp.cmp(&other.header.stamp)
    }
}

impl PartialOrd for TransformStamped {
    fn partial_cmp(&self, other: &TransformStamped)  -> Option<Ordering> {
        Some(self.header.stamp.cmp(&other.header.stamp))
    }
}

#[derive(Debug, Clone)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion
}

impl MessageConverter for Transform {
    type MessageType = ros_msg::Transform;

    fn from_msg(msg: Self::MessageType) -> Transform { todo!() }

    fn toMsg(output: Transform) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    pub seq: u32,
    pub frame_id: String,
    pub stamp: rosrust::Time
}

impl MessageConverter for Header {
    type MessageType = ros_msg::Header;

    fn from_msg(msg: Self::MessageType) -> Header { todo!() }

    fn toMsg(output: Header) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

impl MessageConverter for Vector3 {
    type MessageType = ros_msg::Vector3;

    fn from_msg(msg: Self::MessageType) -> Vector3 { todo!() }

    fn toMsg(output: Vector3) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64
}

impl MessageConverter for Quaternion {
    type MessageType = ros_msg::Quaternion;

    fn from_msg(msg: Self::MessageType) -> Quaternion { todo!() }

    fn toMsg(output: Quaternion) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

impl MessageConverter for TFMessage {
    type MessageType = ros_msg::TFMessage;

    fn from_msg(msg: Self::MessageType) -> TFMessage { todo!() }

    fn toMsg(output: TFMessage) -> Self::MessageType { todo!() }
}