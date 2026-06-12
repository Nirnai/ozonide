use crate::topics;
use crate::msgs::{ActuatorCommand, VehicleState};
use crate::traits::{Controller, SetpointSource};

pub async fn control_task<C, S>(controller: &mut C, setpoint_source: &mut S)
where
    C: Controller,
    S: SetpointSource<Setpoint = C::Setpoint>,
{
    let mut vehicle_state_subscriber = topics::subscriber::<VehicleState>();
    let actuator_command_publisher = topics::publisher::<ActuatorCommand>();

    loop {
        let state = vehicle_state_subscriber.changed().await;
        let setpoint = setpoint_source.latest();
        let control_demand = controller.step(&state, &setpoint);
        actuator_command_publisher.publish(control_demand);
    }
}
