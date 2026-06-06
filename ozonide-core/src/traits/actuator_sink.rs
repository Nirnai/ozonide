
use crate::msgs::ActuatorCommand;

#[allow(async_fn_in_trait)]
pub trait ActuatorSink {
    async fn init(&mut self);
    async fn write(&mut self, command: &ActuatorCommand);
}