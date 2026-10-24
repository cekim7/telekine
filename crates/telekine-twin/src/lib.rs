//! `telekine-twin`
//!
//! Provides realistic digital twin abstractions for physical devices (e.g., Device A, Device B).
//! These virtual devices can be used to simulate remote control by AI/LLMs from point A to B,
//! eventually bridging to environments like MuJoCo for full physics simulation.

use telekine_proto::BoundedIntent;

/// Represents a generic virtual device in a digital twin environment.
pub trait VirtualDevice {
    fn id(&self) -> &str;
    fn process_intent(&mut self, intent: BoundedIntent) -> bool;
    fn get_state(&self) -> Vec<f32>;

    /// Indicates whether this mock device requires validation against a physical device.
    fn requires_real_device_validation(&self) -> bool {
        true
    }
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

    fn process_intent(&mut self, intent: BoundedIntent) -> bool {
        match intent {
            BoundedIntent::MoveToBounded { target_pos_q16, .. } => {
                // Mock execution of movement
                self.position[0] = (target_pos_q16 as f32) / 65536.0;
                self.state_active = true;
                true
            }
            BoundedIntent::EmergencyStop => {
                self.state_active = false;
                true
            }
            _ => false, // unsupported for mock A
        }
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

    fn process_intent(&mut self, intent: BoundedIntent) -> bool {
        match intent {
            BoundedIntent::SetActuationEffortBounded { target_effort_q16, max_effort_pct, .. } => {
                // Mock effort translation
                self.torque = (target_effort_q16 as f32) / 65536.0 * (max_effort_pct as f32 / 100.0);
                true
            }
            BoundedIntent::EmergencyStop => {
                self.torque = 0.0;
                true
            }
            _ => false,
        }
    }

    fn get_state(&self) -> Vec<f32> {
        self.position.to_vec()
    }
}

/// A mock digital twin of a specific use-case purpose device: a Factory Machine.
#[derive(Debug)]
pub struct FactoryMachine {
    pub conveyer_speed: f32,
    pub active: bool,
}

impl FactoryMachine {
    pub fn new() -> Self {
        Self {
            conveyer_speed: 0.0,
            active: false,
        }
    }
}

impl VirtualDevice for FactoryMachine {
    fn id(&self) -> &str {
        "Factory_Machine_01"
    }

    fn process_intent(&mut self, intent: BoundedIntent) -> bool {
        match intent {
            BoundedIntent::SetVelocityBounded { target_vel_q16, max_accel_pct, .. } => {
                let bounded_speed = (target_vel_q16 as f32) / 65536.0 * (max_accel_pct as f32 / 100.0);
                self.conveyer_speed = bounded_speed;
                self.active = true;
                true
            }
            BoundedIntent::EmergencyStop => {
                self.conveyer_speed = 0.0;
                self.active = false;
                true
            }
            _ => false,
        }
    }

    fn get_state(&self) -> Vec<f32> {
        vec![self.conveyer_speed, if self.active { 1.0 } else { 0.0 }]
    }

    // Automatically inherits requires_real_device_validation() -> true
}

/// A mock simulation environment representing MuJoCo or similar physics engines.
pub struct MujocoEnv {
    pub devices: std::collections::HashMap<String, Box<dyn VirtualDevice>>,
}

impl MujocoEnv {
    pub fn new() -> Self {
        Self {
            devices: std::collections::HashMap::new(),
        }
    }

    pub fn register_device(&mut self, device: Box<dyn VirtualDevice>) {
        self.devices.insert(device.id().to_string(), device);
    }

    pub fn send_intent(&mut self, device_id: &str, intent: BoundedIntent) -> bool {
        if let Some(device) = self.devices.get_mut(device_id) {
            device.process_intent(intent)
        } else {
            false
        }
    }
}

/// A mock AI Agent that can orchestrate intents within the simulation.
pub struct AiAgent {
    pub identity: String,
}

impl AiAgent {
    pub fn new(identity: &str) -> Self {
        Self {
            identity: identity.to_string(),
        }
    }

    pub fn command_device(&self, env: &mut MujocoEnv, device_id: &str, intent: BoundedIntent) -> bool {
        // In reality, this would involve LLM tool calling that eventually emits a BoundedIntent.
        env.send_intent(device_id, intent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_a() {
        let mut dev_a = DeviceA { position: [0.0, 0.0, 0.0], state_active: false };
        assert_eq!(dev_a.id(), "Device_A");
        let intent = BoundedIntent::move_to_deg(45.0, 50, 100, 1000);
        let processed = dev_a.process_intent(intent);
        assert!(processed);
        assert!(dev_a.state_active);
        assert_eq!(dev_a.position[0], 45.0);
    }

    #[test]
    fn test_factory_machine_requires_validation() {
        let mut machine = FactoryMachine::new();
        assert_eq!(machine.id(), "Factory_Machine_01");

        // Assert that a newly created specific device mock demands physical validation
        assert!(machine.requires_real_device_validation());

        let intent = BoundedIntent::SetVelocityBounded {
            target_vel_q16: (10.0 * 65536.0) as i32,
            max_accel_pct: 50,
            max_torque_pct: 100,
            lease_timeout_ms: 1000,
        };

        assert!(machine.process_intent(intent));
        let state = machine.get_state();
        assert_eq!(state[0], 5.0); // 10.0 * 50%
        assert_eq!(state[1], 1.0); // Active
    }

    #[test]
    fn test_simulation_environment_and_ai_agent() {
        let mut env = MujocoEnv::new();
        let dev_a = DeviceA { position: [0.0, 0.0, 0.0], state_active: false };
        env.register_device(Box::new(dev_a));

        let ai = AiAgent::new("LLM_Controller_1");

        let intent = BoundedIntent::move_to_deg(90.0, 20, 50, 500);
        let success = ai.command_device(&mut env, "Device_A", intent);
        assert!(success);

        // Ensure state changed via the simulation abstraction layer
        let state = env.devices.get("Device_A").unwrap().get_state();
        assert_eq!(state[0], 90.0);
    }
}
