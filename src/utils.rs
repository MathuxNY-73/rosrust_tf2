use rosrust_msg as msg;

use crate::transforms;
use crate::core::TransformStamped;


/// Calculates the inverse of a ros transform
pub fn get_inverse(transform: TransformStamped) -> TransformStamped {
    
    let m_transform = to_transform(transform.clone());
    let inverse = transforms::invert_transform(&m_transform);

    let inv = msg::geometry_msgs::TransformStamped {
        child_frame_id: transform.header.frame_id.clone(),
        header: msg::std_msgs::Header {
            frame_id: transform.child_frame_id.clone(),
            stamp: transform.header.stamp,
            seq: transform.header.seq
        },
        transform: msg::geometry_msgs::Transform{
            rotation: msg::geometry_msgs::Quaternion{
                x: inverse.orientation.x, y:inverse.orientation.y, z: inverse.orientation.z, w: inverse.orientation.w
            },
            translation: msg::geometry_msgs::Vector3{
                x: inverse.position.x, y: inverse.position.y, z: inverse.position.z
            }
        }
    };
    TransformStamped(inv)
}


pub fn to_transform(transform: TransformStamped) -> transforms::Transform {
    transforms::Transform {
        orientation: transforms::Quaternion{
            x: transform.transform.rotation.x,
            y: transform.transform.rotation.y,
            z: transform.transform.rotation.z,
            w: transform.transform.rotation.w,
        },
        position: transforms::Position{
            x: transform.transform.translation.x,
            y: transform.transform.translation.y,
            z: transform.transform.translation.z
        }
    }
}


pub fn to_transform_stamped(transform: transforms::Transform, from: std::string::String, to: std::string::String, time: rosrust::Time) -> TransformStamped {
    let transform_stamped_msg = msg::geometry_msgs::TransformStamped {
        child_frame_id: to.clone(),
        header: msg::std_msgs::Header {
            frame_id: from.clone(),
            stamp: time,
            seq: 0
        },
        transform: msg::geometry_msgs::Transform{
            rotation: msg::geometry_msgs::Quaternion{
                x: transform.orientation.x, y:transform.orientation.y, z: transform.orientation.z, w: transform.orientation.w
            },
            translation: msg::geometry_msgs::Vector3{
                x: transform.position.x, y: transform.position.y, z: transform.position.z
            }
        }
    };
    TransformStamped(transform_stamped_msg)
}


pub fn get_nanos(dur: rosrust::Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}