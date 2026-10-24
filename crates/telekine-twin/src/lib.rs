//! `telekine-twin`
//!
//! Provides realistic digital twin abstractions for physical devices (e.g., Device A, Device B).
//! These virtual devices can be used to simulate remote control by AI/LLMs from point A to B,
//! eventually bridging to environments like MuJoCo for full physics simulation.

/// Represents a generic virtual device in a digital twin environment.
pub trait VirtualDevice {
    fn id(&self) -> &str;
    fn process_intent(&mut self, intent: &[u8]) -> bool;
    fn get_state(&self) -> Vec<f32>;
}

/// A mock implementation of a specific digital twin (e.g., Device A)
#[derive(Debug)]
pub struct DeviceA {
    pub position: [f32; 3],
    pub state_active: bool,
}

impl VirtualDevice for DeviceA {
    fn id(&self) -> &str {
        "Device_A"
    }

    fn process_intent(&mut self, _intent: &[u8]) -> bool {
        self.state_active = true;
        true
    }

    fn get_state(&self) -> Vec<f32> {
        self.position.to_vec()
    }
}

/// A mock implementation of another specific digital twin (e.g., Device B)
#[derive(Debug)]
pub struct DeviceB {
    pub position: [f32; 3],
    pub torque: f32,
}

impl VirtualDevice for DeviceB {
    fn id(&self) -> &str {
        "Device_B"
    }

    fn process_intent(&mut self, _intent: &[u8]) -> bool {
        self.torque = 50.0;
        true
    }

    fn get_state(&self) -> Vec<f32> {
        self.position.to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_a() {
        let mut dev_a = DeviceA { position: [0.0, 0.0, 0.0], state_active: false };
        assert_eq!(dev_a.id(), "Device_A");
        dev_a.process_intent(&[]);
        assert!(dev_a.state_active);
    }
}
