//! # `telekine-py`
//!
//! C-ABI / PyO3-ready bridge and AI Agent Tool Schema generator exposing Telekine's
//! **Demo 1 Foundational Unit Discovery, Access Claiming, and Bounded Intents**
//! (`Servo`, `Stepper`, `DcMotor`) to Python LLM orchestrators.

use telekine_host::{DiscoveredUnitInfo, SwarmHubCoordinator};
use telekine_proto::{BoundedIntent, Priority, RedundancyMode, UnitElementKind};

/// JSON Schema definitions for AI Agent tool-calling (MCP / OpenAI / Gemini function declarations)
/// supporting Demo 1's 4-step workflow:
/// 1. `telekine_discover_units`
/// 2. `telekine_claim_unit_access`
/// 3. `telekine_dispatch_bounded_intent`
/// 4. `telekine_release_unit_access`
pub const AI_AGENT_TOOL_SCHEMA_JSON: &str = r#"[
  {
    "name": "telekine_discover_units",
    "description": "Broadcast a discovery probe on Master Manager Channel 1 to find nearby compatible Telekine Foundational Units (SERVO, STEPPER, DC_MOTOR) and check whether they are IDLE_AVAILABLE or CLAIMED_BY_HUB.",
    "parameters": {
      "type": "object",
      "properties": {
        "filter_kind": {
          "type": "string",
          "enum": ["ALL", "SERVO", "STEPPER", "DC_MOTOR"],
          "description": "Optional filter to discover only Servo, Stepper, or DC Motor units."
        }
      }
    }
  },
  {
    "name": "telekine_claim_unit_access",
    "description": "Claim exclusive control access to a discovered Servo, Stepper, or DC Motor unit for a bounded lease window and assign it to the Hub's high-speed Worker Channel.",
    "parameters": {
      "type": "object",
      "properties": {
        "target_id": { "type": "integer", "minimum": 0, "maximum": 65533 },
        "access_lease_ms": { "type": "integer", "minimum": 100, "maximum": 10000 }
      },
      "required": ["target_id", "access_lease_ms"]
    }
  },
  {
    "name": "telekine_dispatch_bounded_intent",
    "description": "Dispatch a hardware-bounded physical goal state to a claimed Servo, Stepper, or DC Motor unit over dual-redundant ESP-NOW (2.4 GHz) + Raw LoRa (915 MHz).",
    "parameters": {
      "type": "object",
      "properties": {
        "target_id": { "type": "integer", "minimum": 0, "maximum": 65535 },
        "angle_deg": { "type": "number", "description": "Target bounded goal angle/position in degrees." },
        "max_speed_pct": { "type": "integer", "minimum": 1, "maximum": 100 },
        "max_torque_pct": { "type": "integer", "minimum": 1, "maximum": 100 },
        "lease_timeout_ms": { "type": "integer", "minimum": 50, "maximum": 5000 }
      },
      "required": ["target_id", "angle_deg", "max_speed_pct", "max_torque_pct", "lease_timeout_ms"]
    }
  },
  {
    "name": "telekine_release_unit_access",
    "description": "Release a claimed Servo, Stepper, or DC Motor unit back to safe idle on Master Channel 1 when the AI Agent finishes using it.",
    "parameters": {
      "type": "object",
      "properties": {
        "target_id": { "type": "integer", "minimum": 0, "maximum": 65533 }
      },
      "required": ["target_id"]
    }
  }
]"#;

/// High-level FFI handle for Python / C AI Agents to discover, claim, control, and release
/// Foundational Units (`Servo`, `Stepper`, `DcMotor`).
pub struct TelekineAgentBridge {
    pub coordinator: SwarmHubCoordinator,
}

impl TelekineAgentBridge {
    pub fn new(hub_id: u8, master_key: [u8; 16]) -> Self {
        Self {
            coordinator: SwarmHubCoordinator::new(hub_id, master_key, 6),
        }
    }

    pub fn discover_units(&mut self, filter_kind: UnitElementKind) -> Vec<[u8; 32]> {
        self.coordinator
            .broadcast_discovery_probe(filter_kind)
            .into_iter()
            .map(|tx| tx.frame.0)
            .collect()
    }

    pub fn list_units(&self, filter_kind: Option<UnitElementKind>) -> Vec<DiscoveredUnitInfo> {
        self.coordinator.list_discovered_units(filter_kind)
    }

    pub fn claim_unit(
        &mut self,
        now_ms: u32,
        target_id: u16,
        access_lease_ms: u16,
    ) -> Result<Vec<[u8; 32]>, &'static str> {
        let txs = self
            .coordinator
            .claim_unit_access(now_ms, target_id, access_lease_ms)?;
        Ok(txs.into_iter().map(|tx| tx.frame.0).collect())
    }

    pub fn release_unit(
        &mut self,
        now_ms: u32,
        target_id: u16,
    ) -> Result<Vec<[u8; 32]>, &'static str> {
        let txs = self.coordinator.release_unit_access(now_ms, target_id)?;
        Ok(txs.into_iter().map(|tx| tx.frame.0).collect())
    }

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
