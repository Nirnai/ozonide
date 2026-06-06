use crate::actuator_simulated::ActuatorSimulated;

#[embassy_executor::task]
pub async fn actuator_task(
    actuator: &'static mut ActuatorSimulated,
) {
    ozonide_core::tasks::actuator_task(actuator).await;
}
