# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Project Overview

BiguaSim is a high-fidelity robotics simulator for aerial, surface, and underwater vehicles, built on Unreal Engine 5 (forked from BYU PCCL's Holodeck). This repo is the **Python client layer** (`bs-client`). The UE5 binary server and ROS 2 package are separate components.

**Publication**: MATEUS et al., BiguaSim, ICAR 2025, pp. 169–174.

---

## Setup & Installation

```bash
# Option A: Conda (recommended)
conda env create -f environment.yml
conda activate biguasim
pip install .

# Option B: pip only
pip install .

# Development (editable) install — use this when modifying src/
pip install --editable .

# Docker (GPU required)
docker run -it --privileged --ipc=host --gpus all --runtime=nvidia \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility \
  mgmateus/hydrone:biguasim-base
```

Requires Python ≥ 3.10 and Linux or Windows (macOS not supported).

---

## Commands

```bash
# Run all tests
pytest

# Run a single test file
pytest tests/agents/test_torpedo.py

# Run a single test by name
pytest tests/agents/test_torpedo.py::test_rudders_sterns_motor_speed

# Run a test category
pytest tests/agents
pytest tests/sensors
pytest tests/scenarios

# Run with tox (isolated virtualenv per category)
tox -e py310-agents
tox -e py310-sensors
tox -e py310-scenarios

# Install/manage simulation world packages
python -c "import biguasim; biguasim.install('PackageName')"
python -c "import biguasim; print(biguasim.packagemanager.installed_packages())"

# Build Sphinx docs
python docmaker.py

# Publish package
bash publish.sh
```

Tests require a running UE5 binary and an installed world package. `HOLODECKPATH` env var overrides the default package path (`~/.local/share/biguasim/1.0.0/`).

---

## Architecture

### Client–Server IPC via Shared Memory

The Python client communicates with the UE5 binary exclusively through **POSIX shared memory** and **semaphores** (no network socket). Key mechanism:

- Shared memory files live at `/dev/shm/HOLODECK_MEM{uuid}_{key}` (Linux)
- Two semaphores synchronize turns: `HOLODECK_SEMAPHORE_SERVER{uuid}` and `HOLODECK_SEMAPHORE_CLIENT{uuid}`
- `BiguaSimClient.release()` signals the UE5 server to take a step; `acquire()` blocks until UE5 finishes
- Each sensor and agent action buffer is a numpy array backed by a named shmem block (`Shmem` in `shmem.py`)
- Commands (spawn, sensor add/remove, world commands) are serialized to JSON and written into a dedicated shmem command buffer via `CommandCenter`

### The Step Loop

```
env.step(cmd)
  └─ dynamics_model.step(state, cmd, dt)   # Python-side physics → actuator commands
  └─ agent.act(action)                     # write action to shmem buffer
  └─ env.tick()
       └─ CommandCenter.handle_buffer()    # flush queued JSON commands
       └─ client.release()                 # signal UE5 to advance one tick
       └─ client.acquire()                 # wait for UE5 to finish
       └─ read sensor buffers from shmem   # build state dict
```

### Dynamics Submodule

`src/biguasim/dynamics/` is a **git submodule**. It contains vehicle physics implemented entirely in PyTorch (no UE5 involvement):

| Module | Vehicle class |
|--------|--------------|
| `uuv.py` | `HexaCopterFiveDoF` (BlueROV2), `OctaCopterSixDoF` (BlueROVHeavy), `TorpedoAUV` |
| `uav.py` | `QuadCopterX` (DjiMatrice) |
| `usv.py` | `Catamaran` (BlueBoat) |
| `base_model.py` | `VehicleModel` ABC; `@map_states` decorator unpacks shmem state arrays into `{x, v, q, w}` tensors |
| `control.py` | `BatchedPIDController` (batched, GPU-compatible) |
| `agents.py` | Concrete vehicle configs (`_params` dicts) + `ModelsFactory` |

`ModelsFactory.build_model(agent_type)` maps string type names to dynamics classes. The model's `step(state, cmd, dt)` returns actuator commands (rotor speeds or fin angles) that are written back into the UE5 shmem action buffer.

### Agent & Sensor Registration

- `AgentDefinition._type_keys` maps string agent names to Python agent classes
- `SensorFactory.build_sensor()` allocates the correct shmem block per sensor type
- Every agent **must** have a `DynamicsSensor` configured with `{"UseCOM": True, "UseRPY": False}` — this is enforced in `_load_scenario()` and is required for the dynamics model to parse state

### Batch Agents

Setting `dynamics.batch_size > 1` in the scenario config spawns `N` separate UE5 agents named `{agent_name}-id0` … `{agent_name}-idN-1`. The state returned groups them under the base name as a list indexed by batch position.

### Package Manager

World binary packages are ZIP archives hosted at `BACKEND_URL` (currently `http://192.168.2.110:8000/`). Installed packages live under `~/.local/share/biguasim/{version}/worlds/{package_name}/`. Each package has a `config.json` describing worlds, and per-world scenario `.json` files.

---

## Scenario Configuration

Scenarios are Python dicts (or `.json` files) passed to `biguasim.make()` or `BiguaSimEnvironment`. Required keys:

```python
cfg = {
    "package_name": "Hydrone",          # installed package
    "world": "Relief-Generic-...",       # world within package
    "main_agent": "agent0",             # agent whose state is returned by default
    "ticks_per_sec": 30,                # simulation tick rate
    "frames_per_sec": True,             # True = match ticks_per_sec
    "agents": [
        {
            "agent_name": "agent0",
            "agent_type": "BlueROV2",   # BlueBoat | BlueROV2 | BlueROVHeavy | DjiMatrice | TorpedoAUV
            "sensors": [
                {
                    "sensor_type": "DynamicsSensor",   # REQUIRED
                    "configuration": {"UseCOM": True, "UseRPY": False}
                },
                # optional additional sensors: RGBCamera, DVLSensor, IMUSensor, etc.
            ],
            "dynamics": {"batch_size": 1},
            "control_abstraction": "cmd_vel",   # see control abstractions below
            "location": [0, 0, -3],
            "rotation": [0, 0, 0],
        }
    ]
}
```

### Control Abstractions (per agent type)

All agents support: `accel`, `cmd_motor_speeds`, `cmd_vel`, `cmd_vel_yaw`, `cmd_pos_yaw`

`TorpedoAUV` additionally supports: `cmd_rudders_sterns_motor_speed`, `cmd_depth_heading_rpm_surge`

### State Format

```python
state = env.step(cmd)
# state[agent_name][batch_idx][sensor_name]  → np.ndarray
state['agent0'][0]['DynamicsSensor']   # shape (19,): [accel(3), vel(3), pos(3), ang_accel(3), ang_vel(3), quat(4)]
state['t']                             # simulation time in seconds
```

---

## Workflow Rules

- **ALWAYS write a plan before starting a task with 3+ steps.**
- Follow official Python conventions/standards (PEPs).
- Before starting any task, **read `docs/lessons.md`** to avoid known mistakes.
- After any correction or mistake, add an entry to `docs/lessons.md`:
  - Date (YYYY-MM-DD), what happened, root cause, rule to prevent recurrence.
- When searching or researching, look for both classic and state-of-the-art academic works.

---

## Self-Improvement Loop

When prompted, update this file with any changes to project structure, APIs, conventions, or architectural decisions so future sessions start with full context.
