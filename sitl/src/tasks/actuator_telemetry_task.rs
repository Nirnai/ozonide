use crate::actuator_telemetry_simulated::ActuatorTelemetrySimulated;

#[embassy_executor::task]
pub async fn actuator_telemetry_task(source: &'static mut ActuatorTelemetrySimulated) {
    ozonide_core::tasks::actuator_telemetry_task(source).await;
}
