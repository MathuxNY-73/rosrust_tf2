
use ndarray::prelude::*;
use ndarray::arr2;
use ndarray_linalg::trace::Trace;
use ndarray_linalg::solve::Inverse;
use nalgebra::geometry;

use crate::msg;


/// Converts a SE3 transformation matrix to a quaternion
pub fn quaternion_from_transform_matrix(a: Array2<f64>) ->  msg::Quaternion {
    let tr = a.trace().unwrap();
    let trace = tr - 1f64;
    if trace > 0f64 {
        let qw = (1f64 + trace).sqrt()/2f64;
        msg::Quaternion{
            x: (a[[2,1]] - a[[1,2]])/(4f64*qw),
            y: (a[[0,2]] - a[[2,0]])/(4f64*qw), 
            z: (a[[1,0]] - a[[0,1]])/(4f64*qw), 
            w: qw
        }
    }
    else if a[[0,0]] > a[[1,1]] && a[[0,0]] > a[[2,2]] {
        let s = 2.0f64 * (1.0f64 + a[[0,0]] - a[[1,1]] - a[[2,2]]).sqrt();
        msg::Quaternion{
            w: (a[[2,1]] - a[[1,2]] ) / s,
            x: 0.25f64 * s,
            y: (a[[0,1]] + a[[1,0]] ) / s,
            z: (a[[0,2]] + a[[2,0]] ) / s
        }
    } else if a[[1,1]] > a[[2,2]] {
        let s = 2.0f64 * (1.0f64 + a[[1,1]] - a[[0,0]] - a[[2,2]]).sqrt();
        msg::Quaternion{
            w: (a[[0,2]] - a[[2,0]] ) / s,
            x: (a[[0,1]] + a[[1,0]] ) / s,
            y: 0.25f64 * s,
            z: (a[[1,2]] + a[[2,1]] ) / s
        }
    } else {
        let s = 2.0f64 * (1.0f64 + a[[2,2]] - a[[0,0]] - a[[1,1]]).sqrt();
        msg::Quaternion{
            w: (a[[1,0]] - a[[0,1]] ) / s,
            x: (a[[0,2]] + a[[2,0]] ) / s,
            y: (a[[1,2]] + a[[2,1]] ) / s,
            z: 0.25f64 * s
        }
    }
}

/// Converts a quaternion to an SE3 matrix
pub fn transform_matrix_from_quaternion(msg: msg::Quaternion) ->  Array2<f64> {
    let s  = (msg.x*msg.x+ msg.y*msg.y + msg.z*msg.z + msg.w*msg.w).sqrt();
    let a =  arr2(&[[1f64 - 2f64*s*(msg.y*msg.y + msg.z*msg.z), 2f64*s*(msg.x*msg.y - msg.z*msg.w), 2f64*s*(msg.x*msg.z + msg.y*msg.w), 0f64],
                    [2f64*s*(msg.x*msg.y - msg.w*msg.z), 1f64 - 2f64*s*(msg.x*msg.x + msg.z*msg.z), 2f64*s*(msg.y*msg.z + msg.w*msg.x), 0f64],
                    [2f64*s*(msg.x*msg.z+ msg.w*msg.y), 2f64*s*(msg.y*msg.z-msg.w*msg.x), 1f64 - 2f64*s*(msg.x*msg.x + msg.y*msg.y), 0f64],
                    [0f64, 0f64, 0f64, 1f64]]);
    a
}

///Converts a transform from xyz translation + quaternion format to an SE3 matrix
pub fn se3_from_transform(transform: msg::Transform) -> Array2<f64> {
    let mut a = transform_matrix_from_quaternion(transform.rotation);
    a[[3,0]] = transform.translation.x;
    a[[3,1]] = transform.translation.y;
    a[[3,2]] = transform.translation.z;
    a
}

///Converts an SE3 matrix to a Transform
pub fn transform_from_se3(a: Array2<f64>) -> msg::Transform {
    msg::Transform {
        translation: msg::Vector3 {
            x: a[[3,0]],
            y: a[[3,1]],
            z: a[[3,2]]
        },
        rotation: quaternion_from_transform_matrix(a)
    }
}

///Get the inverse transform
pub fn invert_transform(transform: msg::Transform) -> msg::Transform {
    let m = se3_from_transform(transform);
    let m_inv = m.inv().unwrap();
    transform_from_se3(m_inv)
}

///Chain multiple transforms together. Takes in a vector of transforms. The vector should be in order of desired transformations
pub fn chain_transforms(transforms: Vec<msg::Transform>) -> msg::Transform {
    let mut final_transform = Array::eye(4);
    for t in transforms {
        let tf = se3_from_transform(t);
        final_transform = tf.dot(&final_transform);
    } 
    transform_from_se3(final_transform)
}

pub fn interpolate(t1: msg::Transform, t2: msg::Transform, weight: f64) -> msg::Transform {
    let r1 = geometry::Quaternion::new(t1.rotation.w, t1.rotation.x, t1.rotation.y, t1.rotation.z);
    let r2 = geometry::Quaternion::new(t2.rotation.w, t2.rotation.x, t2.rotation.y, t2.rotation.z);
    let r1 = geometry::UnitQuaternion::from_quaternion(r1);
    let r2 = geometry::UnitQuaternion::from_quaternion(r2);
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
    fn test_quaternion_identity_basic1(){
        let qt = msg::Quaternion {
            x: 0f64,
            y: 0f64,
            z: 0f64,
            w: 1f64
        };
        let arr = transform_matrix_from_quaternion(qt.clone());
        let q2 = quaternion_from_transform_matrix(arr);
        assert_eq!(qt, q2);
    }

    #[test]
    fn test_quaternion_identity_basic2(){
        let qt = msg::Quaternion{
            x: 0f64,
            y: 0f64,
            z: 1f64,
            w: 0f64
        };
        let arr = transform_matrix_from_quaternion(qt.clone());
        let q2 = quaternion_from_transform_matrix(arr);
        assert_eq!(qt, q2);
    }

    #[test]
    fn test_quaternion_identity_basic3(){
        let qt = msg::Quaternion{
            x: 0f64,
            y: 1f64,
            z: 0f64,
            w: 0f64
        };
        let arr = transform_matrix_from_quaternion(qt.clone());
        let q2 = quaternion_from_transform_matrix(arr);
        assert_eq!(qt, q2);
    }

    #[test]
    fn test_quaternion_identity_basic4(){
        let qt = msg::Quaternion{
            x: 1f64,
            y: 0f64,
            z: 0f64,
            w: 0f64
        };
        let arr = transform_matrix_from_quaternion(qt.clone());
        let q2 = quaternion_from_transform_matrix(arr);
        assert_eq!(qt, q2);
    }

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