
use nalgebra::geometry;
use nalgebra::base::{
    Matrix3,
    Matrix4
};

use crate::msg;

pub fn quaternion_from_transform_matrix(a: Matrix3<f64>) -> msg::Quaternion
{
    let trace = a.trace();
    if trace > 0f64
    {
        let s = 0.5f64 / (1f64 + trace).sqrt();
        msg::Quaternion {
            x: (a[(2, 1)] - a[(1, 2)]) * s,
            y: (a[(0, 2)] - a[(2, 0)]) * s,
            z: (a[(1, 0)] - a[(0, 1)]) * s,
            w: 0.25f64 / s
        }
    }
    else if a[(0, 0)] > a[(1, 1)] && a[(0, 0)] > a[(2, 2)]
    {
        let s = 2f64 * (1f64 + a[(0, 0)] - a[(1, 1)] - a[(2, 2)]).sqrt();
        msg::Quaternion {
            x: 0.25f64 * s,
            y: (a[(0, 1)] + a[(1, 0)]) / s,
            z: (a[(0, 2)] + a[(2, 0)]) / s,
            w: (a[(2, 1)] - a[(1, 2)]) / s,
        }
    }
    else if a[(1, 1)] > a[(2, 2)]
    {
        let s = 2f64 * (1f64 + a[(1, 1)] - a[(0, 0)] - a[(2, 2)]).sqrt();
        msg::Quaternion {
            x: (a[(0, 1)] + a[(1, 0)]) / s,
            y: 0.25f64 * s,
            z: (a[(1, 2)] + a[(2, 1)]) / s,
            w: (a[(0, 2)] - a[(2, 0)] ) / s,
        }
    }
    else
    {
        let s = 2f64 * (1f64 + a[(2, 2)] - a[(0, 0)] - a[(1, 1)]).sqrt();
        msg::Quaternion{
            x: (a[(0, 2)] + a[(2, 0)] ) / s,
            y: (a[(1, 2)] + a[(2, 1)] ) / s,
            z: 0.25f64 * s,
            w: (a[(1, 0)] - a[(0, 1)] ) / s,
        }
    }
}

/// Converts a quaternion to an SE3 matrix
pub fn transform_matrix_from_quaternion(quaternion: msg::Quaternion) ->  Matrix3<f64> {
    let msg::Quaternion { x, y, z, w} = quaternion;

    let s  = (x * x + y * y + z * z + w * w).sqrt();
    let a =  Matrix3::new(
            1f64 - 2f64 * s * (y * y + z * z),
            2f64 * s * (x * y - z * w),
            2f64 * s * (x * z + y * w),
            2f64 * s * (x * y + z * w),
            1f64 - 2f64 * s * (x * x + z * z),
            2f64 * s * (y * z - x * w),
            2f64 * s * (x * z - y * w),
            2f64 * s * (y * z + x * w),
            1f64 - 2f64 * s * (x * x + y * y)
        );
    a
}

///Converts a transform from xyz translation + quaternion format to an SE3 matrix
pub fn se3_from_transform(transform: msg::Transform) -> Matrix4<f64> {
    let matrix = transform_matrix_from_quaternion(transform.rotation);
    let matrix = matrix.insert_column(3, 0f64);
    let mut matrix = matrix.insert_row(3, 0f64);
    matrix[(3, 0)] = transform.translation.x;
    matrix[(3, 1)] = transform.translation.y;
    matrix[(3, 2)] = transform.translation.z;
    matrix[(3, 3)] = 1f64;
    matrix
}

///Converts an SE3 matrix to a Transform
pub fn transform_from_se3(a: Matrix4<f64>) -> msg::Transform
{
    let rotation_matrix = a.fixed_slice::<3,3>(0,0).clone_owned();
    msg::Transform {
        translation: msg::Vector3 {
            x: a[(3, 0)],
            y: a[(3, 1)],
            z: a[(3, 2)]
        },
        rotation: quaternion_from_transform_matrix(rotation_matrix)
    }
}

///Get the inverse transform
pub fn invert_transform(transform: msg::Transform) -> msg::Transform {
    let m = se3_from_transform(transform);
    let m_inv = m.try_inverse().unwrap();
    transform_from_se3(m_inv)
}

///Chain multiple transforms together. Takes in a vector of transforms. The vector should be in order of desired transformations
pub fn chain_transforms(transforms: Vec<msg::Transform>) -> msg::Transform {
    let mut final_transform = Matrix4::identity();
    for t in transforms {
        let tf = se3_from_transform(t);
        final_transform = tf * final_transform;
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
        assert_eq!(arr, Matrix3::identity());
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