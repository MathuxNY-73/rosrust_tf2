use rosrust::Duration;
use crate::buffer::{BufferInterfaceWithTime, BufferInterface};

const DefaultCacheTime: i32 = 10;
const MaxGraphDepth: u32 = 1000;


enum TransformableResult 
{
    TransformAvailable,
    TransformFailure
}


struct BufferCore;


impl BufferCore
{
    fn new(cache_time: Option<Duration>) -> BufferCore {
        BufferCore {}
    }

    fn setTranform() -> () {}
}


impl BufferInterfaceWithTime for BufferCore
{
    fn lookup_transform(_: &str, _: rosrust::Time, _: &str, _: rosrust::Time, _: &str, _: rosrust::Duration) { todo!() }
    fn can_transform(_: &str, _: rosrust::Time, _: &str, _: rosrust::Time, _: &str, _: rosrust::Duration) -> std::result::Result<bool, std::boxed::Box<(dyn std::error::Error + 'static)>> { todo!() }
}


impl BufferInterface for BufferCore 
{    
    fn lookup_transform(_: &str, _: &str, _: rosrust::Time) { todo!() }
    fn can_transform(_: &str, _: &str, _: rosrust::Time, _: rosrust::Duration) -> std::result::Result<bool, std::boxed::Box<(dyn std::error::Error + 'static)>> { todo!() }
    fn transform_from_input<T>(_: T, _: &str, _: std::option::Option<rosrust::Duration>) -> T { todo!() }
    fn transform_to_output<'a, T>(_: &'a T, _: &'a T, _: &str, _: std::option::Option<rosrust::Duration>) -> &'a T { todo!() }
}