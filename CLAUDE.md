# BiguaSim (bs-client)

Python **client library** for a hybrid aerial/water robotics simulator. This repo is Python-only; the physics/rendering run in a **separate UE5 binary** it talks to over shared memory. Vehicle dynamics are computed here in PyTorch. Gym-like API. Forked from BYU Holodeck.

## Commands
- Install: `pip install .`  (dev/editable: `pip install --editable .`; clone with `git submodule update --init --recursive` first)
- Conda env: `conda env create -f environment.yml`
- Download a world: `python -c "import biguasim; biguasim.install('SkyDive')"`
- Test (all): `pytest`   (single: `pytest tests/agents/test_djimatrice.py::test_cmd_vel`)  — needs an installed world binary + GPU
- Lint: `pylint --rcfile=src/biguasim/pylintrc src/biguasim`  (no enforced formatter)
- Publish to public repo: `bash publish.sh "<msg>"`

## Critical rules
- `dynamics/` and `tools/` are **git submodules** (`bs-dynamics`, `bs-tools`) — init them, and commit changes in the submodule repo, not here.
- Adding an agent type requires registering in **both** `agents.py` (`AgentDefinition._type_keys`) and `dynamics/agents.py` (`ModelsFactory._types`) with an identical `agent_type` string.
- All library code lives in `src/biguasim/`. Raise `BiguaSimException` (from `exceptions.py`) for library errors.
- Shared-memory/semaphore names use the legacy `HOLODECK` prefix and must match the UE5 binary — never rename. `Command` parameter order is positional and must match the engine.
- `env.reset()` must be called before `step()`/`tick()`.
- Version lives in `pyproject.toml` **and** `src/biguasim/__init__.py` — bump both. `setup.py` is stale (ignore).
- Linux/Windows only; macOS unsupported.

## Where to look (read before acting)
| When you need to… | Read |
|---|---|
| Understand the system | `ai/architecture.md` |
| Know the stack/versions | `ai/tech-stack.md` |
| Find where code lives | `ai/structure.md` |
| Follow coding standards | `ai/conventions.md` |
| Do a common task | `ai/workflows.md` |
| Use the Python/env API | `ai/api.md` |
| Work on the env/tick loop | `ai/domains/environment.md` |
| Work on agents or sensors | `ai/domains/agents-sensors.md` |
| Work on physics/controllers | `ai/domains/dynamics.md` |
| Work on shared-memory/IPC | `ai/domains/ipc-transport.md` |
| Install/manage worlds | `ai/domains/packagemanager.md` |
| Work on the ArduPilot bridge | `ai/domains/ardubridge.md` |
| Run/write tests | `ai/testing.md` |
| Build, env vars, deploy, publish | `ai/build-deploy.md` |
| Avoid known pitfalls | `ai/gotchas.md` |
| Decode domain jargon | `ai/glossary.md` |

## Notes for agents
- Prefer editing existing patterns (registries, factories, `Command` subclasses) over introducing new ones.
- If a doc conflicts with the code, the code wins — and fix the doc.
- This client can't fully run without the external UE5 binary; many behaviors are only observable against a live world.
