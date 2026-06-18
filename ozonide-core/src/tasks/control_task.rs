use crate::topics;
use crate::msgs::{ActuatorCommand, StateValidity, VehicleState};
use crate::traits::{Controller, SetpointSource};

pub async fn control_task<C, S>(controller: &mut C, setpoint_source: &mut S)
where
    C: Controller,
    S: SetpointSource<Setpoint = C::Setpoint>,
{
    let mut vehicle_state_subscriber = topics::subscriber::<VehicleState>();
    let actuator_command_publisher = topics::publisher::<ActuatorCommand>();

    // Warm-start all signal conditioning filters before the first step.
    // The estimator returns a default (zero) state on its first call, which
    // would leave filters at zero and produce a large spurious thrust
    // increment on step 1. Wait for the first state that carries real IMU
    // data (ATTITUDE flag set), then drive filters to their DC steady state.
    let initial_state = loop {
        let state = vehicle_state_subscriber.changed().await;
        if state.valid.contains(StateValidity::ATTITUDE) {
            break state;
        }
    };
    controller.reset(&initial_state);

    loop {
        let state = vehicle_state_subscriber.changed().await;
        let setpoint = setpoint_source.latest();
        let control_demand = controller.step(&state, &setpoint);
        actuator_command_publisher.publish(control_demand);
    }
}
