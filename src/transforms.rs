use nalgebra::geometry;

use crate::msg;

///Converts a transform from xyz translation + quaternion format to an SE3 matrix
pub fn isometry_from_transform_msg(transform: msg::Transform) -> geometry::Isometry3<f64>
{
    let msg::Quaternion {x: qx, y: qy, z: qz, w: qw} = transform.rotation;
    let msg::Vector3{x: tx, y: ty, z: tz} = transform.translation;

    let qt = geometry::UnitQuaternion::new_normalize(
        geometry::Quaternion::new(qw, qx, qy, qz)
    );
    let trans = geometry::Translation3::new(tx, ty, tz);

    geometry::Isometry3::new(trans.vector, qt.scaled_axis())
}

///Converts an SE3 matrix to a Transform
pub fn transform_msg_from_isometry(isometry: geometry::Isometry3<f64>) -> msg::Transform
{
    let translation_vec = isometry.translation.vector;
    let quaternion = isometry.rotation;

    msg::Transform {
        translation: msg::Vector3 {
            x: translation_vec[0],
            y: translation_vec[1],
            z: translation_vec[2]
        },
        rotation: msg::Quaternion {
            x: quaternion.coords[0],
            y: quaternion.coords[1],
            z: quaternion.coords[2],
            w: quaternion.coords[3]
        }
    }
}

///Get the inverse transform
pub fn invert_transform(transform: msg::Transform) -> msg::Transform {
    let isometry = isometry_from_transform_msg(transform);
    let inverse_isometry = isometry.inverse();
    transform_msg_from_isometry(inverse_isometry)
}

///Chain multiple transforms together. Takes in a vector of transforms. The vector should be in order of desired transformations
pub fn chain_transforms(transforms: Vec<msg::Transform>) -> msg::Transform {
    let final_transform_opt = transforms
        .into_iter()
        .map(isometry_from_transform_msg)
        .reduce(|tf1, tf2| tf1 * tf2);

    if let Some(final_transform) = final_transform_opt
    {
        transform_msg_from_isometry(final_transform)
    }
    else
    {
        panic!("No transforms to chain")
    }
}

pub fn interpolate(t1: msg::Transform, t2: msg::Transform, weight: f64) -> msg::Transform {
    let r1 = geometry::UnitQuaternion::new_normalize(
        geometry::Quaternion::new(t1.rotation.w, t1.rotation.x, t1.rotation.y, t1.rotation.z));
    let r2 = geometry::UnitQuaternion::new_normalize(
        geometry::Quaternion::new(t2.rotation.w, t2.rotation.x, t2.rotation.y, t2.rotation.z));
    let res  = r1.try_slerp(&r2, weight, 1e-9);
    match res {
        Some(qt) => {
            msg::Transform{
                translation: msg::Vector3 {
                    x: t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                    y: t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                    z: t1.translation.z * weight + t2.translation.z * (1.0 - weight) 
                },
                rotation: msg::Quaternion {
                    x: qt.coords[0],
                    y: qt.coords[1],
                    z: qt.coords[2],
                    w: qt.coords[3]
                }
            } 
        }
        None => {
            if weight > 0.5 {
                msg::Transform{
                    translation: msg::Vector3 {
                        x: t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                        y: t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                        z: t1.translation.z * weight + t2.translation.z * (1.0 - weight) 
                    },
                    rotation: t1.rotation.clone()
                }
            }
            else {
                msg::Transform{
                    translation: msg::Vector3 {
                        x: t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                        y: t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                        z: t1.translation.z * weight + t2.translation.z * (1.0 - weight) 
                    },
                    rotation: t2.rotation.clone()
                }
            }
        } 
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_basic_translation_chaining(){
        let tf1 = msg::Transform {
            translation: msg::Vector3{x: 1f64, y: 1f64, z: 0f64},
            rotation: msg::Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let expected_tf = msg::Transform {
            translation: msg::Vector3{x: 2f64, y: 2f64, z: 0f64},
            rotation: msg::Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let transform_chain = vec!(tf1.clone(), tf1);
        let res = chain_transforms(transform_chain);
        assert_eq!(res, expected_tf);
    }

    #[test]
    fn test_basic_interpolation() {
        let tf1 = msg::Transform {
            translation: msg::Vector3{x: 1f64, y: 1f64, z: 0f64},
            rotation: msg::Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let tf2 = msg::Transform {
            translation: msg::Vector3{x: 2f64, y: 2f64, z: 0f64},
            rotation: msg::Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let expected = msg::Transform {
            translation: msg::Vector3{x: 1.5f64, y: 1.5f64, z: 0f64},
            rotation: msg::Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        assert_eq!(interpolate(tf1, tf2, 0.5), expected);
    }
}