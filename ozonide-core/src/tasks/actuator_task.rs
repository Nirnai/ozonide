use crate::topics::ACTUATOR_TOPIC;
use crate::traits::ActuatorSink;


pub async fn actuator_task(sink: &mut impl ActuatorSink){
    sink.init().await;
    #[cfg(feature = "defmt")]
    defmt::info!("Controller ready, entering control loop");
    #[cfg(feature = "log")]
    log::info!("Controller ready, entering control loop");
    let mut subscriber = ACTUATOR_TOPIC.subscriber();

    loop {
        let command = subscriber.changed().await; 
        sink.write(&command).await;
    }
}