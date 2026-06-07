use crate::topics::{CONTROL_DEMAND_TOPIC, VEHICLE_STATE_TOPIC};
use crate::traits::{Controller, SetpointSource};

pub async fn control_task<C, S>(controller: &mut C, setpoint_source: &mut S)
where
    C: Controller,
    S: SetpointSource<Setpoint = C::Setpoint>,
{
    let mut vehicle_state_subscriber = VEHICLE_STATE_TOPIC.subscriber();
    let control_demand_publisher = CONTROL_DEMAND_TOPIC.publisher();

    loop {
        let state = vehicle_state_subscriber.changed().await;
        let setpoint = setpoint_source.next().await;
        let control_demand = controller.update(&state, &setpoint);
        control_demand_publisher.publish(control_demand);
    }
}
