use rosrust::Time;
use rosrust::Duration;
use std::result::Result;


pub trait TransformInterface {
    fn lookup_transform(target_frame: &str, source_frame: &str, time: Time) -> ();
    fn can_transform(target_frame: &str, source_frame: &str, time: Time, timeout: Duration) -> Result<bool, Box<dyn std::error::Error>>;

    fn transform_to_output<'a, T>(input: &'a T, output: &'a T, target_frame: &str, timeout: Option<Duration>) -> &'a T;
    fn transform_from_input<T>(input: T, target: &str, timeout: Option<Duration>) -> T;
}

pub trait TransformWithTimeInterface {
    fn lookup_transform(target_frame: &str, target_time: Time,
                       source_frame: &str, source_time: Time,
                       fixed_frame: &str, timeout: Duration) -> ();
    
    fn can_transform(target_frame: &str, target_time: Time, source_frame: &str, source_time: Time, fixed_frame: &str, 
        timeout: Duration) -> Result<bool, Box<dyn std::error::Error>>;
}
