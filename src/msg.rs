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
    fn to_msg(output: Output) -> Self::MessageType;
}

/**
 * NewType pattern on geometry_msgs::TransformStamped 
 */
#[derive(Debug, Clone, PartialEq)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform
}

impl MessageConverter for TransformStamped {
    type MessageType = ros_msg::TransformStamped;

    fn from_msg(_msg: Self::MessageType) -> TransformStamped { todo!() }

    fn to_msg(_output: TransformStamped) -> Self::MessageType { todo!() }
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

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion
}

impl MessageConverter for Transform {
    type MessageType = ros_msg::Transform;

    fn from_msg(_msg: Self::MessageType) -> Transform { todo!() }

    fn to_msg(_output: Transform) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    pub seq: u32,
    pub frame_id: String,
    pub stamp: rosrust::Time
}

impl MessageConverter for Header {
    type MessageType = ros_msg::Header;

    fn from_msg(_msg: Self::MessageType) -> Header { todo!() }

    fn to_msg(_output: Header) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

impl MessageConverter for Vector3 {
    type MessageType = ros_msg::Vector3;

    fn from_msg(_msg: Self::MessageType) -> Vector3 { todo!() }

    fn to_msg(_output: Vector3) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64
}

impl MessageConverter for Quaternion {
    type MessageType = ros_msg::Quaternion;

    fn from_msg(_msg: Self::MessageType) -> Quaternion { todo!() }

    fn to_msg(_output: Quaternion) -> Self::MessageType { todo!() }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

impl MessageConverter for TFMessage {
    type MessageType = ros_msg::TFMessage;

    fn from_msg(_msg: Self::MessageType) -> TFMessage { todo!() }

    fn to_msg(_output: TFMessage) -> Self::MessageType { todo!() }
}