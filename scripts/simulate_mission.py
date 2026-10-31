"""
Two-drone alert simulation, run in one process.

Alpha has no payload left and reports a high-confidence target on its
return (wayback) leg. The coordinator should alert bravo, which still
has payload. No drone changes its own target status.

Run from the project root:
    python3 scripts/simulate_mission.py
"""

import sys
from pathlib import Path

# Allow running as a script from the project root without installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from nidar.coordinator import Coordinator
from nidar.drone import Drone
from nidar.drone_config import DroneConfig
from nidar.transport import InMemoryTransport


def show(label, message):
    print(f"  {label:<10} {message.sender} -> {message.recipient}: "
          f"{message.msg_type} {message.payload}")


def main():
    transport = InMemoryTransport()
    coordinator = Coordinator(transport)

    alpha_cfg = DroneConfig(drone_id="alpha", capabilities=("scout", "delivery"))
    bravo_cfg = DroneConfig(drone_id="bravo", capabilities=("scout", "delivery"))
    alpha = Drone(alpha_cfg, transport)
    bravo = Drone(bravo_cfg, transport)

    coordinator.register_drone("alpha", payload_capacity=0)
    coordinator.register_drone("bravo", payload_capacity=7)

    print("Step 1: alpha reports its payload is empty")
    alpha.send_to_gcs("scout_status", {"payload_remaining": 0})
    coordinator.process_inbox()

    print("Step 2: alpha reports a high-confidence target on its wayback leg")
    alpha.report_target("t1", 12.9716, 77.5946, 0.95, phase="wayback")
    coordinator.process_inbox()

    print("Step 3: messages delivered to each drone")
    for drone in (alpha, bravo):
        inbox = drone.poll()
        if not inbox:
            print(f"  {drone.drone_id}: no messages")
        for message in inbox:
            show("received", message)

    print("Final state")
    print(f"  alerts sent: {coordinator.alerts}")
    target = coordinator.registry.get("t1")
    print(f"  t1 status: {target.status.value}, assigned_to: {target.assigned_to}")


if __name__ == "__main__":
    main()
