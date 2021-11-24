use crate::transforms;
use crate::msg;


/// Calculates the inverse of a ros transform
pub fn get_inverse(transform: msg::TransformStamped) -> msg::TransformStamped {
    
    let m_transform = to_transform(transform.clone());
    let inverse = transforms::invert_transform(&m_transform);

    let inv = msg::TransformStamped {
        child_frame_id: transform.header.frame_id.clone(),
        header: msg::Header {
            frame_id: transform.child_frame_id.clone(),
            stamp: transform.header.stamp,
            seq: transform.header.seq
        },
        transform: msg::Transform{
            rotation: msg::Quaternion{
                x: inverse.orientation.x, y:inverse.orientation.y, z: inverse.orientation.z, w: inverse.orientation.w
            },
            translation: msg::Vector3{
                x: inverse.position.x, y: inverse.position.y, z: inverse.position.z
            }
        }
    };
    inv
}


pub fn to_transform(transform: msg::TransformStamped) -> transforms::Transform {
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


pub fn to_transform_stamped(transform: transforms::Transform, from: std::string::String, to: std::string::String, time: rosrust::Time) -> msg::TransformStamped {
    let transform_stamped_msg = msg::TransformStamped {
        child_frame_id: to.clone(),
        header: msg::Header {
            frame_id: from.clone(),
            stamp: time,
            seq: 0
        },
        transform: msg::Transform{
            rotation: msg::Quaternion{
                x: transform.orientation.x, y:transform.orientation.y, z: transform.orientation.z, w: transform.orientation.w
            },
            translation: msg::Vector3{
                x: transform.position.x, y: transform.position.y, z: transform.position.z
            }
        }
    };
    transform_stamped_msg
}


pub fn get_nanos(dur: rosrust::Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}