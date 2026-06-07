use crate::control::allocate_control;
use crate::topics::{ACTUATOR_TOPIC, CONTROL_DEMAND_TOPIC};
use crate::traits::ActuatorSink;

pub async fn actuator_task(sink: &mut impl ActuatorSink) {
    sink.init().await;
    #[cfg(feature = "defmt")]
    defmt::info!("Actuator task ready, entering loop");
    #[cfg(feature = "log")]
    log::info!("Actuator task ready, entering loop");
    let mut subscriber = CONTROL_DEMAND_TOPIC.subscriber();
    let actuator_publisher = ACTUATOR_TOPIC.publisher();

    loop {
        let demand = subscriber.changed().await;
        let command = allocate_control(&demand);
        actuator_publisher.publish(command);
        sink.write(&command).await;
    }
}
