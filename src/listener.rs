use std::sync::{Arc, RwLock};

use rosrust;
use rosrust_msg as msg;

use crate::core::{TransformStamped, TfError, TransformInterface, TransformWithTimeInterface};
use crate::buffer::TfBuffer;


///This struct tries to be the same as the C++ version of `TransformListener`. Use this struct to lookup transforms.
/// 
/// Example usage:
/// 
/// ```ignore
/// fn main() {
///     rosrust::init("listener");
///     let listener = TfListener::new();
///     
///     let rate = rosrust::rate(1.0);
///     while rosrust::is_ok() {
///         let tf = listener.lookup_transform("camera", "base_link", ros::Time::now());
///         println!("{:?}", tf);
///         rate.sleep();
///     }
/// }
/// ```
/// Do note that unlike the C++ variant of the TfListener, only one TfListener can be created at a time. Like its C++ counterpart,
/// it must be scoped to exist through the lifetime of the program. One way to do this is using an `Arc` or `RwLock`.
pub struct TfListener {
    buffer: Arc<RwLock<TfBuffer>>,
    static_subscriber: rosrust::Subscriber,
    dynamic_subscriber:  rosrust::Subscriber, 
}

impl TfListener {

    /// Create a new TfListener
    pub fn new() -> Self {
        let buff = RwLock::new(TfBuffer::new());
        let arc = Arc::new(buff);
        let r1 = arc.clone();
        let _subscriber_tf = rosrust::subscribe("tf", 100, move |v: msg::tf2_msgs::TFMessage| {
            r1.write().unwrap().handle_incoming_transforms(v, true);
        }).unwrap();

        let r2 = arc.clone();
        let _subscriber_tf_static = rosrust::subscribe("tf_static", 100, move |v: msg::tf2_msgs::TFMessage| {
            r2.write().unwrap().handle_incoming_transforms(v, true);
        }).unwrap();
        
        TfListener {
            buffer: arc.clone(),
            static_subscriber: _subscriber_tf_static,
            dynamic_subscriber: _subscriber_tf
        }
    }
}

impl TransformInterface for TfListener {
    /// Looks up a transform within the tree at a given time.
    fn lookup_transform(&self, from: &str, to: &str, time: rosrust::Time) ->  Result<TransformStamped,TfError> {
        self.buffer.read().unwrap().lookup_transform(from, to, time)
    }

    // TODO(MathuxNY-73) implement those methods
    fn can_transform(&self, target_frame: &str, source_frame: &str, time: rosrust::Time, timeout: rosrust::Duration) -> Result<bool, TfError> {todo!()}

    fn transform_to_output<'a, T>(&self, input: &'a T, output: &'a T, target_frame: &str, timeout: Option<rosrust::Duration>) -> &'a T {todo!()}
    fn transform_from_input<T>(&self, input: T, target: &str, timeout: Option<rosrust::Duration>) -> T {todo!()}
}

impl TransformWithTimeInterface for TfListener {
    /// Looks up a transform within the tree at a given time.
    fn lookup_transform_with_time_travel(&self, from: &str, time1: rosrust::Time, to: &str, time2: rosrust::Time, fixed_frame: &str, timeout: rosrust::Duration) ->  Result<TransformStamped,TfError> {
        self.buffer.read().unwrap().lookup_transform_with_time_travel(from, time1, to, time2, fixed_frame, timeout)
    }

    fn can_transform_with_time_travel(&self, target_frame: &str, target_time: rosrust::Time, source_frame: &str, source_time: rosrust::Time, fixed_frame: &str, 
        timeout: rosrust::Duration) -> Result<bool, TfError> {todo!()}
}
