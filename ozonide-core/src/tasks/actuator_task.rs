use crate::msgs::ActuatorCommand;
use crate::topics;
use crate::traits::ActuatorSink;

pub async fn actuator_task(sink: &mut impl ActuatorSink) {
    sink.init().await;
    #[cfg(feature = "defmt")]
    defmt::info!("Actuator task ready, entering loop");
    #[cfg(feature = "log")]
    log::info!("Actuator task ready, entering loop");
    let mut actuator_command = topics::subscriber::<ActuatorCommand>();

    loop {
        let command = actuator_command.changed().await;
        sink.write(&command).await;
    }
}
