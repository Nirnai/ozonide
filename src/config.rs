pub struct SensorConfig {
    pub role: &'static str,
    pub name: &'static str,
    pub interface: &'static str,
    pub pins: &'static [PinConfig],
    pub params: &'static [ParamConfig],
}

pub struct PinConfig {
    pub name: &'static str,
    pub pin: &'static str,
}

pub struct ParamConfig {
    pub name: &'static str,
    pub value: &'static str,
}

impl SensorConfig {
    pub fn pin(&self, name: &str) -> &'static str {
        self.pins.iter()
            .find(|p| p.name == name)
            .unwrap_or_else(|| panic!("Pin '{}' not found for {}", name, self.name))
            .pin
    }

    pub fn param(&self, name: &str) -> &'static str {
        self.params.iter()
            .find(|p| p.name == name)
            .unwrap_or_else(|| panic!("Param '{}' not found for {}", name, self.name))
            .value
    }
}

include!(concat!(env!("OUT_DIR"), "/generated_config.rs"));