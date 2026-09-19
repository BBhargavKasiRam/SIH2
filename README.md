# FleetMind — Edge-AI Based Distributed Fleet Coordination for AMRs in Smart Warehouses

> **Smart India Hackathon 2024 — Problem Statement ID: 26123**
> *Organization: Bharat Electronics Limited (BEL) | Category: Software | Theme: Smart Automation*

---

## Overview

FleetMind is a **decentralized, Edge-AI-powered coordination and collision-avoidance framework** for multi-robot fleets of Autonomous Mobile Robots (AMRs) operating in dynamic warehouse environments. It is designed to run locally on edge hardware (NVIDIA Jetson Nano, Raspberry Pi, or similar) without depending on a centralized cloud server.

### Key Capabilities

| Capability | Description |
|---|---|
| **Decentralized P2P Communication** | AMRs share position, velocity, intent, and task state directly via DDS peer-to-peer messaging — no centralized server required |
| **Edge-AI Collision Prediction** | ML-augmented trajectory prediction runs onboard each robot, with deterministic kinematics fallback |
| **Multi-Factor Right-of-Way Negotiation** | Distributed priority arbitration (task priority → wait-time → lexicographic tiebreak) with hysteresis anti-oscillation |
| **Distributed Task Allocation** | Decentralized cost-based bidding for task assignment — robots evaluate tasks locally and submit bids |
| **Dynamic Task Reassignment** | Automatic re-offering of orphaned tasks when a robot becomes unavailable |
| **Deadlock Detection & Recovery** | Watchdog timer detects prolonged yielding and triggers RMF replan alerts |
| **Fleet Dashboard** | Lightweight web-based monitoring dashboard — monitoring only, not a single point of failure |
| **Open-RMF Integration** | Full compatibility with ROS 2 / Open-RMF schedule, fleet adapters, and traffic management |

---

## Architecture

```
┌────────────────────────────────────────────┐
│           Fleet Dashboard                   │
│        Visualization / Status               │
│      (Monitoring Only — NOT SPOF)           │
└───────────────┬────────────────────────────┘
                │ (subscribes to fleet topics)
                │
    ┌───────────┴───────────┐
    │   Local Communication  │ (DDS / ROS 2 P2P Topics)
    │                        │
┌───┴────┐  ◄──────────►  ┌─┴──────┐  ◄──────────►  ┌─────────┐
│ AMR 1  │                 │ AMR 2  │                 │ AMR 3   │
│        │                 │        │                 │         │
│Edge AI │                 │Edge AI │                 │Edge AI  │
│Planner │                 │Planner │                 │Planner  │
│Local   │                 │Local   │                 │Local    │
│Decision│                 │Decision│                 │Decision │
└────────┘                 └────────┘                 └─────────┘
```

**Each AMR runs its own Edge-AI stack locally:**
- `edge_ai_node` — Collision prediction, risk evaluation, right-of-way negotiation
- `rmf_adapter` — Translates safety decisions into velocity overrides and RMF PauseRequests
- `distributed_task_allocator` — Evaluates and bids on task offers
- Shared via P2P DDS topics — no mandatory centralized server

---

## ROS 2 Package Structure

```
rmf_ws/src/
├── fleetmind_msgs/           # Custom ROS 2 message interfaces
│   └── msg/
│       ├── PeerIntent.msg        # Robot position, velocity, intent broadcast
│       ├── SafetyDecision.msg    # Yield/Continue/Monitor decisions
│       ├── RobotState.msg        # Full robot state for dashboard
│       ├── Heartbeat.msg         # Periodic health heartbeat
│       ├── P2PEnvelope.msg       # Generic peer-to-peer message envelope
│       ├── ObstacleAlert.msg     # Dynamic obstacle notifications
│       ├── TaskOffer.msg         # Decentralized task announcement
│       └── TaskBid.msg           # Robot bid for offered task
│
├── rmf_fleetmind_edge_ai/    # Core Edge-AI coordination framework
│   ├── rmf_fleetmind_edge_ai/
│   │   ├── edge_ai_node.py           # Onboard AI: collision prediction + arbitration
│   │   ├── collision_predictor.py    # ML + deterministic trajectory prediction
│   │   ├── distributed_task_allocator.py  # Decentralized task bidding
│   │   ├── fleet_coordinator.py      # Fleet telemetry + fault recovery
│   │   ├── rmf_adapter.py           # RMF PauseRequest/cmd_vel bridge
│   │   └── mock_robot_sim.py        # Lightweight mock robot simulator
│   ├── launch/
│   │   ├── edge_ai_fleet.launch.py       # Universal fleet launch
│   │   ├── warehouse_edge_ai.launch.py   # ★ Primary SIH demo (3 warehouse AMRs)
│   │   ├── office_edge_ai.launch.py      # Office demo (3 AMRs)
│   │   ├── clinic_edge_ai.launch.py      # Clinic demo (3+ AMRs, multi-fleet)
│   │   ├── hotel_edge_ai.launch.py       # Hotel demo
│   │   └── campus_edge_ai.launch.py      # Campus demo (3 AMRs)
│   ├── config/
│   │   └── edge_ai_params.yaml           # Tunable parameters
│   ├── models/
│   │   └── best_model.joblib             # Pre-trained ML collision model
│   └── test/
│       ├── test_collision_predictor.py
│       ├── test_cooperative_yielding.py
│       └── test_task_allocator.py
│
├── fleetmind_dashboard/      # Web-based fleet monitoring dashboard
│   ├── fleetmind_dashboard/
│   │   └── dashboard_node.py         # ROS 2 → WebSocket/SSE bridge
│   ├── static/
│   │   ├── index.html                # Dashboard UI
│   │   ├── style.css                 # Premium dark theme
│   │   └── dashboard.js              # Real-time SSE client
│   └── launch/
│       └── dashboard.launch.py
│
└── rmf_demos/                # Open-RMF demo maps, fleet adapters, configs
    ├── rmf_demos/                # Launch files and fleet configs
    ├── rmf_demos_maps/           # Navigation graphs and building maps
    ├── rmf_demos_fleet_adapter/  # Mock fleet adapter
    ├── rmf_demos_gz/             # Gazebo simulation launch files
    └── ...
```

---

## Topic Architecture

| Topic | Message Type | Direction | Purpose |
|---|---|---|---|
| `/fleetmind/p2p_intent` | `PeerIntent` | Pub/Sub (all AMRs) | P2P position/velocity/intent broadcast |
| `/fleetmind/safety_decision` | `SafetyDecision` | Pub/Sub (all AMRs) | Safety decisions shared globally |
| `/<robot_id>/safety_decision` | `SafetyDecision` | Pub (per AMR) | Per-robot safety decision |
| `/fleetmind/heartbeats` | `Heartbeat` | Pub (per AMR) | Health/liveness heartbeats |
| `/fleetmind/robot_states` | `RobotState` | Pub (per AMR) | Full robot state for dashboard |
| `/fleetmind/task_offers` | `TaskOffer` | Pub/Sub | Decentralized task announcements |
| `/fleetmind/task_bids` | `TaskBid` | Pub (per AMR) | Cost-based task bids |
| `/fleetmind/obstacle_alerts` | `ObstacleAlert` | Pub/Sub | Dynamic obstacle notifications |
| `/fleetmind/fleet_diagnostics` | `String` | Pub (coordinator) | Fleet telemetry summary |
| `/fleet_states` | `FleetState` (RMF) | Sub | RMF fleet state integration |
| `/<robot_id>/odom` | `Odometry` | Sub | Direct robot odometry |
| `/<robot_id>/cmd_vel` | `Twist` | Pub | Velocity override (yield stop) |

---

## Edge-AI Component

### What Intelligence Runs Locally

Each robot's `edge_ai_node` runs a complete AI decision pipeline onboard:

1. **ML Collision Prediction** (`collision_predictor.py`)
   - Pre-trained scikit-learn model (`best_model.joblib`) classifies risk as SAFE/WARNING/CRITICAL
   - Features: relative position, velocity, heading, deterministic TTC
   - **Graceful fallback**: If ML libraries are unavailable (e.g., on constrained hardware), the system seamlessly falls back to deterministic kinematic trajectory simulation
   - **Safety consensus**: Deterministic CRITICAL overrides ML SAFE (fail-safe)

2. **Distributed Right-of-Way Negotiation**
   - Multi-factor priority arbitration without any central coordinator
   - Anti-oscillation hysteresis window prevents yield chattering

3. **Decentralized Task Cost Estimation**
   - Each robot locally computes bid costs based on distance, battery, workload, and urgency
   - No centralized task assignment server needed

### Cloud Independence

The entire coordination stack runs without any cloud connectivity:
- All P2P communication is via DDS (ROS 2 default middleware)
- ML inference is lightweight (scikit-learn, ~2KB model)
- No internet access required during operation

### Edge Hardware Compatibility

| Component | Jetson Nano | Raspberry Pi 4 |
|---|---|---|
| ROS 2 Humble | ✅ | ✅ |
| Edge AI Node | ✅ | ✅ |
| ML Inference | ✅ (fast) | ✅ (deterministic fallback) |
| Dashboard | ✅ | ✅ |

---

## Prerequisites

- **ROS 2 Humble** (or Jazzy/Rolling)
- **Python 3.10+**
- **Open-RMF** packages (installed via binary or source)
- Optional: `scikit-learn`, `joblib`, `pandas` (for ML collision prediction)

## Build

```bash
cd ~/Documents/PROJECTS/rmf_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

## Launch (Examples)

### Primary SIH Demo — Smart Warehouse (3 AMRs + Dashboard)
```bash
ros2 launch rmf_fleetmind_edge_ai warehouse_edge_ai.launch.py
# Dashboard available at http://localhost:7860
```

### Office Demo (3 AMRs + Dashboard)
```bash
ros2 launch rmf_fleetmind_edge_ai office_edge_ai.launch.py
```

### Clinic Demo (3+ AMRs, Multi-Fleet)
```bash
ros2 launch rmf_fleetmind_edge_ai clinic_edge_ai.launch.py
```

### Campus Demo (3 AMRs)
```bash
ros2 launch rmf_fleetmind_edge_ai campus_edge_ai.launch.py
```

### Dashboard Only (connect to running system)
```bash
ros2 launch fleetmind_dashboard dashboard.launch.py http_port:=7860
```

### Disable Dashboard
```bash
ros2 launch rmf_fleetmind_edge_ai warehouse_edge_ai.launch.py dashboard:=false
```

---

## Tests

```bash
cd ~/Documents/PROJECTS/rmf_ws
colcon test --packages-select rmf_fleetmind_edge_ai
colcon test-result --verbose
```

Unit tests cover:
- Head-on collision detection
- Diverging safe path classification
- Cross-traffic warning detection
- ML predictor graceful fallback
- Task priority arbitration
- Wait-time starvation prevention
- Lexicographic tiebreak determinism
- Anti-oscillation hysteresis
- Cost-based task allocation

---

## License

Apache License 2.0
