use rosrust::Time;
use rosrust::Duration;
use std::result::Result;
use std::error::Error;
use std::fmt;

use crate::msg;


/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast, 
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture, 
    /// There is no path between the from and to frame.
    CouldNotFindTransform
}

impl fmt::Display for TfError {
    fn fmt(&self, _f: &mut fmt::Formatter) -> fmt::Result {todo!()}
}

impl Error for TfError {}

pub trait TransformInterface {
    fn lookup_transform(&self, target_frame: &str, source_frame: &str, time: Time) -> Result<msg::TransformStamped, Box<dyn Error>>;
    fn can_transform(&self, target_frame: &str, source_frame: &str, time: Time, timeout: Duration) -> Result<bool, Box<dyn Error>>;

    fn transform_to_output<'a, T>(&self, input: &'a T, output: &'a T, target_frame: &str, timeout: Option<Duration>) -> &'a T;
    fn transform_from_input<T>(&self, input: T, target: &str, timeout: Option<Duration>) -> T;
}

pub trait TransformWithTimeInterface {
    fn lookup_transform_with_time_travel(&self, target_frame: &str, target_time: Time,
                       source_frame: &str, source_time: Time,
                       fixed_frame: &str, timeout: Duration) -> Result<msg::TransformStamped, Box<dyn Error>>;

    fn can_transform_with_time_travel(&self, target_frame: &str, target_time: Time, source_frame: &str, source_time: Time, fixed_frame: &str, 
        timeout: Duration) -> Result<bool, Box<dyn Error>>;
}