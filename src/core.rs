use rosrust::Time;
use rosrust::Duration;
use std::result::Result;
use std::cmp::Ordering;
use std::ops::{Deref, DerefMut};

use rosrust_msg::geometry_msgs;

/**
 * NewType pattern on geometry_msgs::TransformStamped 
 */
#[derive(Debug, Clone)]
pub struct TransformStamped(pub geometry_msgs::TransformStamped);

impl Deref for TransformStamped {
    type Target = geometry_msgs::TransformStamped;

    fn deref(self: &'_ Self) -> &'_ Self::Target {
        &self.0
    }
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


/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast, 
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture, 
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    CouldNotAcquireLock
}


pub trait TransformInterface {
    fn lookup_transform(&self, target_frame: &str, source_frame: &str, time: Time) -> Result<TransformStamped, TfError>;
    fn can_transform(&self, target_frame: &str, source_frame: &str, time: Time, timeout: Duration) -> Result<bool, TfError>;

    fn transform_to_output<'a, T>(&self, input: &'a T, output: &'a T, target_frame: &str, timeout: Option<Duration>) -> &'a T;
    fn transform_from_input<T>(&self, input: T, target: &str, timeout: Option<Duration>) -> T;
}

pub trait TransformWithTimeInterface {
    fn lookup_transform_with_time_travel(&self, target_frame: &str, target_time: Time,
                       source_frame: &str, source_time: Time,
                       fixed_frame: &str, timeout: Duration) -> Result<TransformStamped, TfError>;
    
    fn can_transform_with_time_travel(&self, target_frame: &str, target_time: Time, source_frame: &str, source_time: Time, fixed_frame: &str, 
        timeout: Duration) -> Result<bool, TfError>;
}