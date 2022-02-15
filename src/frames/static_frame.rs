use crate::frames::frame_visitor::{
    MutableFrameVisitor,
    ImmutableFrameVisitor,
    GetDataVisitor,
    InsertVisitor,
    VisitableFrame
};
use crate::msg;


#[derive(Clone, Debug)]
pub struct StaticFrame
{
    transform: msg::TransformStamped
}

impl StaticFrame
{
    pub fn new(transform: msg::TransformStamped) -> Self
    {
        StaticFrame {
            transform: transform
        }
    }

    pub fn get_parent(&self) -> &str
    {
        &self.transform.header.frame_id
    }

    pub fn get_data(&self) -> &msg::TransformStamped
    {
        &self.transform
    }

    pub fn insert_data(&mut self, transform: msg::TransformStamped)
    {
        self.transform = transform;
    }
}

impl VisitableFrame for StaticFrame
{
    fn accept_get_data(&self, visitor: GetDataVisitor) -> <GetDataVisitor as ImmutableFrameVisitor>::Output
    {
        visitor.do_for_static_frame(self)
    }

    fn accept_insertion(&mut self, visitor: InsertVisitor) -> <InsertVisitor as MutableFrameVisitor>::Output
    {
        visitor.do_for_static_frame(self)
    }
}