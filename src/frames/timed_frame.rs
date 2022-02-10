use std::cmp::Reverse;

use rosrust;

use crate::frames::frame_visitor::{
    MutableFrameVisitor,
    ImmutableFrameVisitor,
    GetDataVisitor,
    InsertVisitor,
    VisitableFrame
};
use crate::msg;
use crate::transforms;

const DEFAULT_MAX_STORAGE_TIME: i64 = 10i64 * 100000000i64;

#[derive(Clone, Debug)]
pub struct TimedFrame {
    transform_cache: Vec<msg::TransformStamped>,
    max_storage_time: rosrust::Duration
}

impl TimedFrame 
{
    pub fn new(max_storage_time: rosrust::Duration) -> Self
    {
        TimedFrame {
            transform_cache: Vec::new(),
            max_storage_time: max_storage_time
        }
    }

    fn find_closest(&self, target_time: rosrust::Time) -> (Option<&msg::TransformStamped>, Option<&msg::TransformStamped>)
    {
        // TODO[MathuxNY-73] This method should return a Result
        match self.transform_cache.len()
        {
            0 => {
                (None, None)
            },
            1 => {
                // We do unwrap here as we already matched the len of the collection to 1.
                // ! This could be a race condition here. Let us take care of it in a future PR.
                // ! NOTE: The race condition exists if the collection is not locked.
                let first_transform = self.transform_cache.first().unwrap();
                
                if target_time == rosrust::Time::default() || first_transform.header.stamp == target_time
                {
                    (Some(first_transform), None)
                }
                else 
                { 
                    (None, None)
                    // Return Err
                }
            },
            _ => {
                // This is equivalent to C++ lower_bound because we make sure there are no duplicated time stamps. See https://doc.rust-lang.org/std/vec/struct.Vec.html#method.binary_search_by_key
                match self.transform_cache.binary_search_by_key(&Reverse(target_time), |tf| Reverse(tf.header.stamp))
                {
                    Ok(tf_index) => {
                        (self.transform_cache.get(tf_index), None)
                    },
                    Err(insertion_index) if insertion_index == 0 => (None, None),
                    Err(insertion_index) if insertion_index >= self.transform_cache.len() => (None, None),
                    Err(insertion_index) => (self.transform_cache.get(insertion_index), self.transform_cache.get(insertion_index -  1))
                }
            }
        }
    }

    fn interpolate(older_tf: &msg::TransformStamped, younger_tf: &msg::TransformStamped, time: rosrust::Time) -> msg::TransformStamped
    {
        if older_tf.header.stamp == younger_tf.header.stamp
        {
            younger_tf.clone()
        }
        else
        {
            let ratio = ((time - older_tf.header.stamp).sec as f64) / ((younger_tf.header.stamp - older_tf.header.stamp).sec as f64);

            let interpolated_transform = transforms::interpolate(older_tf.transform.clone(),
                                                                younger_tf.transform.clone(), ratio);

            msg::TransformStamped {
                header: msg::Header {
                    stamp: time,
                    frame_id: older_tf.header.frame_id.clone(),
                    ..msg::Header::default()
                },
                transform: interpolated_transform,
                child_frame_id: older_tf.child_frame_id.clone()
            }
        }
    }

    pub fn get_latest_time_and_parent(&self) -> (rosrust::Time, Option<&str>)
    {
        match self.transform_cache.first()
        {
            Some(frame) => {
                (frame.header.stamp, Some(&frame.header.frame_id))
            },
            None => {
                (rosrust::Time::default(), None)
            }
        }
    }

    pub fn get_parent(&self, time: rosrust::Time) -> Option<&str>
    {
        // TODO Handle error cases here and propagate
        match self.find_closest(time) {
            (None, _) => None,
            (Some(older_tf), _) => Some(&older_tf.header.frame_id)
        }
    }

    pub fn get_data(&self, time: rosrust::Time) -> Option<msg::TransformStamped>
    {
        // TODO Handle error cases here and propagate
        match self.find_closest(time)
        {
            (None, _) => None,
            (Some(older_tf), Some(younger_tf)) 
                if older_tf.header.frame_id == younger_tf.header.frame_id => Some(TimedFrame::interpolate(older_tf, younger_tf, time)),
            (Some(older_tf), _) => Some(older_tf.clone()),
        }
    }

    pub fn insert_data(&mut self, transform: msg::TransformStamped) -> Result<(), String>
    {
        match self.transform_cache.first() {
            Some(tf) if tf.header.stamp > transform.header.stamp + self.max_storage_time =>
                Err(format!("TF_OLD_DATA ignoring data from the past (Possible reasons are listed at http://wiki.ros.org/tf/Errors%%20explained)")),
            _ => {
                match self.transform_cache
                          .iter()
                          .enumerate()
                          .find(|(_, tf)| tf.header.stamp <= transform.header.stamp)
                {
                    Some((_, tf)) if tf.header.stamp == transform.header.stamp => 
                        Err(format!("TF_REPEATED_DATA ignoring data with redundant timestamp")),
                    Some((insertion_index, _)) => {
                        self.transform_cache.insert(insertion_index, transform);
                        Ok(())
                    },
                    _ => {
                        self.transform_cache.push(transform);
                        Ok(())
                    },
                }
            },
        }
        // TODO Prune the list to remove old transforms
    }
}

impl Default for TimedFrame
{
    fn default() -> Self
    {
        Self::new(rosrust::Duration::from_nanos(DEFAULT_MAX_STORAGE_TIME))
    }
}

impl VisitableFrame for TimedFrame
{
    fn accept_get_data(&self, visitor: GetDataVisitor) -> <GetDataVisitor as ImmutableFrameVisitor>::Output
    {
        visitor.do_for_time_frame(self)
    }

    fn accept_insertion(&mut self, visitor: InsertVisitor) -> <InsertVisitor as MutableFrameVisitor>::Output
    {
        visitor.do_for_time_frame(self)
    }
}


#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_timed_get_too_old_parent()
    {
        let timed_frame = TimedFrame {
            transform_cache: (5..10).map(|t| msg::TransformStamped {
                header: msg::Header {
                    stamp: rosrust::Time::from_seconds(t),
                    frame_id: format!("parent_{}", t),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }).rev().collect(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        let parent = timed_frame.get_parent(rosrust::Time::from_seconds(1));
        assert_eq!(parent, None);
    }

    #[test]
    fn test_timed_get_too_young_parent()
    {
        let timed_frame = TimedFrame {
            transform_cache: (5..10).map(|t| msg::TransformStamped {
                header: msg::Header {
                    stamp: rosrust::Time::from_seconds(t),
                    frame_id: format!("parent_{}", t),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }).rev().collect(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        let parent = timed_frame.get_parent(rosrust::Time::from_seconds(12));
        assert_eq!(parent, None);
    }

    #[test]
    fn test_timed_get_inexisting_parent()
    {
        let timed_frame = TimedFrame {
            transform_cache: Vec::new(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        let parent = timed_frame.get_parent(rosrust::Time::from_seconds(1));
        assert_eq!(parent, None);
    }

    #[test]
    fn test_timed_get_parent()
    {
        let timed_frame = TimedFrame {
            transform_cache: (0..10).map(|t| msg::TransformStamped {
                header: msg::Header {
                    stamp: rosrust::Time::from_seconds(t),
                    frame_id: format!("parent_{}", t),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }).rev().collect(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        let parent = timed_frame.get_parent(rosrust::Time::from_seconds(3));
        assert_eq!(parent, Some("parent_3"));

        let parent = timed_frame.get_parent(rosrust::Time::from_nanos(3.5e9 as i64));
        assert_eq!(parent, Some("parent_3"));

        let timed_frame = TimedFrame {
            transform_cache: (0..3).map(|t| msg::TransformStamped {
                header: msg::Header {
                    stamp: rosrust::Time::from_seconds(t),
                    frame_id: String::from("parent"),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }).rev().collect(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        let parent = timed_frame.get_parent(rosrust::Time::from_nanos(1.5e9 as i64));
        assert_eq!(parent, Some("parent"));
    }

    #[test]
    fn test_timed_get_closest_with_no_transforms()
    {
        let timed_frame = TimedFrame {
            transform_cache: Vec::new(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        assert_eq!(timed_frame.find_closest(rosrust::Time::default()), (None, None));
    }

    #[test]
    fn test_timed_get_closest_with_1_transform()
    {
        let timed_frame = TimedFrame {
            transform_cache: [msg::TransformStamped {
                header: msg::Header {
                    stamp: rosrust::Time::from_seconds(73),
                    frame_id: String::from("parent"),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }].to_vec(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        // Tests that when passing Time Zero we get back the transform
        let (older, younger) = timed_frame.find_closest(rosrust::Time::default());
        assert_eq!(younger, None);

        let older = older.unwrap();
        assert_eq!(older.header.frame_id, "parent");
        assert_eq!(older.header.stamp, rosrust::Time::from_seconds(73));

        // Tests that when passing the Time of the only tranform we get it returned
        let (older, younger) = timed_frame.find_closest(rosrust::Time::from_seconds(73));
        assert_eq!(younger, None);

        let older = older.unwrap();
        assert_eq!(older.header.frame_id, "parent");
        assert_eq!(older.header.stamp, rosrust::Time::from_seconds(73));

        // Tests that when passing arbitrary Time we return nothing
        let (older, younger) = timed_frame.find_closest(rosrust::Time::from_seconds(74));
        assert_eq!(younger, None);
        assert_eq!(older, None);
    }

    #[test]
    fn test_timed_get_closest_with_more_than_2_transform()
    {
        let timed_frame = TimedFrame {
            transform_cache: (3..7).map(|t| msg::TransformStamped {
                header: msg::Header {
                    stamp: rosrust::Time::from_seconds(t),
                    frame_id: format!("parent_{}", t),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }).rev().collect(),
            max_storage_time: rosrust::Duration::from_seconds(10),
        };

        // Test find closest with Time Zero (which is too old)
        let (older, younger) = timed_frame.find_closest(rosrust::Time::default());
        assert_eq!(younger, None);
        assert_eq!(older, None);

        // Test find closest with too old date
        let (older, younger) = timed_frame.find_closest(rosrust::Time::from_seconds(2));
        assert_eq!(younger, None);
        assert_eq!(older, None);

        // Test find closest with too young date
        let (older, younger) = timed_frame.find_closest(rosrust::Time::from_seconds(7));
        assert_eq!(younger, None);
        assert_eq!(older, None);

        // Test find closest with exact date
        let (older, younger) = timed_frame.find_closest(rosrust::Time::from_seconds(4));
        assert_eq!(younger, None);

        let older = older.unwrap();
        assert_eq!(older.header.frame_id, "parent_4");
        assert_eq!(older.header.stamp, rosrust::Time::from_seconds(4));

        // Test find closest interpolated 
        let (older, younger) = timed_frame.find_closest(rosrust::Time::from_nanos(4.5e9 as i64));
        let younger = younger.unwrap();
        let older = older.unwrap();

        assert_eq!(younger.header.frame_id, "parent_5");
        assert_eq!(younger.header.stamp, rosrust::Time::from_seconds(5));
        assert_eq!(older.header.frame_id, "parent_4");
        assert_eq!(older.header.stamp, rosrust::Time::from_seconds(4));
    }

    #[test]
    fn test_timed_insert_data()
    {
        let mut timed_frame = TimedFrame::default();

        let res = timed_frame.insert_data(
            msg::TransformStamped {
                header: msg::Header {
                    frame_id: String::from("parent_1"),
                    stamp: rosrust::Time::from_nanos(73e9 as i64 + DEFAULT_MAX_STORAGE_TIME),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }
        );

        assert_eq!(res, Ok(()));
        assert_eq!(timed_frame.transform_cache.len(), 1);
        assert_eq!(timed_frame.transform_cache.first().unwrap().header.frame_id, "parent_1");

        // Test insertion of too old data (passed the max storage time)
        let res = timed_frame.insert_data(msg::TransformStamped {
                header: msg::Header {
                    frame_id: String::from("parent_2"),
                    stamp: rosrust::Time::from_seconds(72),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }
        );

        match res {
            Err(err_msg) => assert!(err_msg.starts_with("TF_OLD_DATA")),
            _ => panic!("Assertion failure: expected Err got Ok")
        }

        // Test insertion of older transform
        let res = timed_frame.insert_data(msg::TransformStamped {
                header: msg::Header {
                    frame_id: String::from("parent_3"),
                    stamp: rosrust::Time::from_nanos(72e9 as i64 + DEFAULT_MAX_STORAGE_TIME),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }
        );

        assert_eq!(res, Ok(()));

        let expected_frame_ids = [
            String::from("parent_1"),
            String::from("parent_3")
        ];

        for (expected, actual) in expected_frame_ids.iter().zip(timed_frame.transform_cache.iter().map(|tf| &tf.header.frame_id))
        {
            assert_eq!(expected, actual);
        }

        // Test insertion of younger transform
        let res = timed_frame.insert_data(msg::TransformStamped {
                header: msg::Header {
                    frame_id: String::from("parent_4"),
                    stamp: rosrust::Time::from_nanos(74e9 as i64 + DEFAULT_MAX_STORAGE_TIME),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }
        );

        assert_eq!(res, Ok(()));

        let expected_frame_ids = [
            String::from("parent_4"),
            String::from("parent_1"),
            String::from("parent_3")
        ];

        for (expected, actual) in expected_frame_ids.iter().zip(timed_frame.transform_cache.iter().map(|tf| &tf.header.frame_id))
        {
            assert_eq!(expected, actual);
        }

        // Test insertion of same time transform
        let res = timed_frame.insert_data(msg::TransformStamped {
                header: msg::Header {
                    frame_id: String::from("parent_5"),
                    stamp: rosrust::Time::from_nanos(74e9 as i64 + DEFAULT_MAX_STORAGE_TIME),
                    ..msg::Header::default()
                },
                ..msg::TransformStamped::default()
            }
        );

        match res {
            Err(err_msg) => assert!(err_msg.starts_with("TF_REPEATED_DATA")),
            _ => panic!("Assertion failure: expected Err got Ok")
        }
        assert_eq!(timed_frame.transform_cache.len(), 3);
    }
}