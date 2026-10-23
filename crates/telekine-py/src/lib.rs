//! # `telekine-py`
//!
//! C-ABI / PyO3-ready bridge and AI Agent Tool Schema generator exposing Telekine's
//! **Bounded Intents** to Python LLM orchestrators (OpenAI, Gemini, Anthropic, MCP, ROS 2).
//!
//! Ensures an LLM can never synthesize an unbounded open-loop command: every tool schema
//! requires `max_speed_pct`, `max_torque_pct`, and `lease_timeout_ms`.

use telekine_host::SwarmHubCoordinator;
use telekine_proto::{BoundedIntent, Priority, RedundancyMode};

/// JSON Schema definition for AI Agent tool-calling (MCP / OpenAI / Gemini function declarations)
/// enforcing bounded goal states rather than open-loop toggles.
pub const AI_AGENT_TOOL_SCHEMA_JSON: &str = r#"{
  "name": "telekine_dispatch_bounded_intent",
  "description": "Dispatch a hardware-bounded physical goal state to one actuator (0..65533) or the entire 1,000-node swarm (65535) over dual-redundant ESP-NOW (2.4 GHz) + Raw LoRa (915 MHz). Never sends open-loop toggles; the actuator closes the 1 kHz loop locally and enters fail-safe if the watchdog lease expires.",
  "parameters": {
    "type": "object",
    "properties": {
      "target_id": {
        "type": "integer",
        "minimum": 0,
        "maximum": 65535,
        "description": "2-byte connectionless node address (0..65533 for unicast, 65535 for Triple-Burst swarm broadcast)."
      },
      "angle_deg": {
        "type": "number",
        "description": "Target bounded goal angle in degrees."
      },
      "max_speed_pct": {
        "type": "integer",
        "minimum": 1,
        "maximum": 100,
        "description": "Maximum allowed speed percentage (automatically throttled in LoRa Limp Mode)."
      },
      "max_torque_pct": {
        "type": "integer",
        "minimum": 1,
        "maximum": 100,
        "description": "Maximum allowed torque/effort percentage."
      },
      "lease_timeout_ms": {
        "type": "integer",
        "minimum": 50,
        "maximum": 5000,
        "description": "Dual-Link Dead-Man's Switch lease duration in milliseconds (typical: 250-1000 ms)."
      },
      "redundancy_mode": {
        "type": "string",
        "enum": ["HOT_HOT_TWIN_CAST", "HOT_WARM_FAILOVER", "TRIPLE_BURST_BROADCAST"]
      },
      "priority": {
        "type": "string",
        "enum": ["BACKGROUND_BK", "NORMAL_BE", "HIGH_VI", "CRITICAL_VO", "VIP_EMERGENCY"]
      },
      "exec_at_tick_ms": {
        "type": "integer",
        "description": "Optional synchronized millisecond timestamp for Sync-and-Fire swarm execution."
      }
    },
    "required": ["target_id", "angle_deg", "max_speed_pct", "max_torque_pct", "lease_timeout_ms"]
  }
}"#;

/// High-level FFI handle for Python / C callers to drive `SwarmHubCoordinator`.
pub struct TelekineAgentBridge {
    pub coordinator: SwarmHubCoordinator,
}

impl TelekineAgentBridge {
    pub fn new(hub_id: u8, master_key: [u8; 16]) -> Self {
        Self {
            coordinator: SwarmHubCoordinator::new(hub_id, master_key, 6),
        }
    }

    /// Dispatch a bounded move command from an AI agent tool call and return the raw
    /// 32-byte encrypted frames ready for USB-CDC / UART / UDP transmission.
    pub fn dispatch_agent_move(
        &mut self,
        now_ms: u32,
        target_id: u16,
        angle_deg: f32,
        max_speed_pct: u8,
        max_torque_pct: u8,
        lease_timeout_ms: u16,
        priority: Priority,
        redundancy: RedundancyMode,
        exec_at_tick_ms: Option<u32>,
    ) -> Result<Vec<[u8; 32]>, &'static str> {
        let intent = BoundedIntent::move_to_deg(
            angle_deg,
            max_speed_pct,
            max_torque_pct,
            lease_timeout_ms,
        );
        let txs = self.coordinator.dispatch_unicast(
            now_ms,
            target_id,
            intent,
            priority,
            redundancy,
            exec_at_tick_ms,
        )?;
        Ok(txs.into_iter().map(|tx| tx.frame.0).collect())
    }
}
