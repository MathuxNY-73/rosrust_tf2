use crate::msg;
use crate::frames::frame;

pub trait ImmutableFrameVisitor
{
    type Output;

    fn do_for_time_frame(&self, frame: &frame::TimedFrame) -> Self::Output;
    fn do_for_static_frame(&self, frame: &frame::StaticFrame) -> Self::Output;
}

pub trait MutableFrameVisitor
{
    type Output;

    fn do_for_time_frame(self, frame: &mut frame::TimedFrame) -> Self::Output;
    fn do_for_static_frame(self, frame: &mut frame::StaticFrame) -> Self::Output;
}

pub struct InsertVisitor
{
    transform: msg::TransformStamped
}

impl InsertVisitor
{
    pub fn new(transform: msg::TransformStamped) -> Self
    {
        InsertVisitor {
            transform
        }
    }
}

impl MutableFrameVisitor for InsertVisitor
{
    type Output = Result<(), String>;

    fn do_for_static_frame(self, frame: &mut frame::StaticFrame) -> Self::Output
    {
        frame.insert_data(self.transform);
        Ok(())
    }

    fn do_for_time_frame(self, frame: &mut frame::TimedFrame) -> Self::Output
    {
        frame.insert_data(self.transform)
    }
}

pub struct GetDataVisitor
{
    time: Option<rosrust::Time>
}

impl GetDataVisitor
{
    pub fn new(time: Option<rosrust::Time>) -> Self
    {
        GetDataVisitor {
            time
        }
    }
}

impl ImmutableFrameVisitor for GetDataVisitor
{
    type Output = Option<msg::TransformStamped>;

    fn do_for_static_frame(&self, frame: &frame::StaticFrame) -> Self::Output
    {
        Some(frame.get_data().clone())
    }
    fn do_for_time_frame(&self, frame: &frame::TimedFrame) -> Self::Output
    {
        self.time.map(|time| frame.get_data(time)).flatten()
    }

}

pub trait VisitableFrame
{
    fn accept_insertion(&mut self, visitor: InsertVisitor) -> <InsertVisitor as MutableFrameVisitor>::Output;
    fn accept_get_data(&self, visitor: GetDataVisitor) -> <GetDataVisitor as ImmutableFrameVisitor>::Output;
}