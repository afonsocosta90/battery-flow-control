# Battery Thermal MPC

**Predictive thermal control for an immersion-cooled 24s13p lithium-ion battery module.**

A C++20 simulation project demonstrating Model Predictive Control versus PID on a data-grounded battery thermal management problem. The plant is a 24-node lumped-capacitance model with an algebraic serial coolant chain energy balance. The MPC uses a from-scratch single-shooting gradient-descent solver — no external QP libraries.

> Portfolio piece targeting Planning & Control Software Engineer roles.

---

## Headline Results

Two quantifiable claims on the same physical model and hardware constraints:

### A — Step-transient constraint satisfaction (1C → 5C at t = 120 s)

| Controller | Peak T_cell | Violations | ∫ṁ² dt |
|------------|------------|------------|--------|
| **MPC**    | **35.0 °C** | **0**      | 10.3   |
| PID        | 36.7 °C    | **1 078**  | 9.8    |

MPC holds T_cell < 35 °C throughout. PID (well-tuned for steady-state) lags the step: cells breach the constraint for **107.8 s** before PID integrator recovers. No preview of the step is given to either controller — MPC's 20-step horizon alone is responsible for the difference.

### B — Pump energy efficiency (sustained 5C discharge, 600 s)

| Controller | Peak T_cell | Violations | ∫ṁ² dt       |
|------------|------------|------------|--------------|
| **MPC**    | 35.0 °C    | **0**      | **50.5**     |
| PID        | 34.7 °C    | 0          | 150.3        |

MPC uses **66% less pump energy** while achieving equal or better constraint satisfaction. PID tracks an unnecessarily cool setpoint (32 °C); MPC tracks the constraint boundary (34.9 °C) with minimum flow, spending pump energy only where thermodynamically necessary.

### C — Rising ambient (T_inlet 25 → 27 °C ramp, 5C)

| Controller | Peak T_cell | Violations | ∫ṁ² dt |
|------------|------------|------------|--------|
| **MPC**    | 35.0 °C    | **0**      | **35.6** |
| PID        | 34.8 °C    | 0          | 91.7   |

MPC scales flow to exactly compensate the rising inlet, using **61% less pump energy** than a PID tracking the same scenario at a conservative setpoint.

---

## Quickstart

```bash
# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTest_DIR=/opt/homebrew/lib/cmake/GTest
cmake --build . -j

# Run tests (25 tests, ~0.2 s)
ctest --output-on-failure

# Run scenarios (from project root, not build/)
./build/btm config/default.yaml pid
./build/btm config/default.yaml mpc
./build/btm config/scenario_step_transient.yaml pid
./build/btm config/scenario_step_transient.yaml mpc
./build/btm config/scenario_rising_ambient.yaml pid
./build/btm config/scenario_rising_ambient.yaml mpc

# Generate comparison plots
python3 scripts/compare_controllers.py \
    --pid results/const5c_pid.csv \
    --mpc results/const5c_mpc.csv \
    --out results/plot_const5c.png
```

**Dependencies**: yaml-cpp, GoogleTest, Python 3 + pandas + matplotlib (plots only).

**IDE support**: a `compile_commands.json` symlink at the project root points to `build/compile_commands.json` so clangd and VS Code find headers automatically after the first CMake configure.

---

## Architecture

Seven modules in a strict DAG — no cycles, no globals, no singletons:

```
core  ──►  config, model, solver
config, model, solver  ──►  control
config  ──►  scenario
control, model, scenario  ──►  sim
sim  ──►  main
```

| Module | Role |
|--------|------|
| `core` | Strong physical types (`Temperature`, `MassFlowRate`, …), constants. Zero deps. |
| `config` | YAML load + validation. Throws `std::runtime_error` with the named offending field. |
| `model` | **Pure function** `(ThermalState, ṁ, I, T_inlet, dt) → ThermalState`. 24-node explicit Euler + algebraic successive-substitution coolant chain. Owns no state. |
| `solver` | `GradientDescentSolver` — finite-difference gradients, box projection, warm-start. |
| `control` | C++20 `Controller` concept + `PidController` + `MpcController`. The only stateful layer. |
| `scenario` | Three discharge profiles. Returns `std::function` closures for current and inlet temperature. |
| `sim` | `Simulator<Controller>` — templated time-stepping harness + CSV logger. Zero virtuals. |

### Key design choices

- **`Controller` is a C++20 concept**, not a base class. `compute_command(ThermalState, Current, Temperature, Duration) → MassFlowRate`. Zero virtuals in the hot path.
- **No-preview policy**: MPC receives the current `I_cell` and `T_inlet` and assumes them constant over the 20-step horizon. No oracle for future disturbances.
- **Serial coolant asymmetry preserved**: the 24th node always sees pre-heated coolant — this is the core physical reason MPC outperforms PID. Any simplification that erases it (uniform coolant, single lumped node) is an architectural error.
- **Template implementation in `.ipp`**: `Simulator<Controller>` is defined in `include/btm/sim/simulator.ipp`, `#include`d at the bottom of `simulator.hpp` to keep headers readable while ensuring the body is visible at instantiation sites.

---

## Test Suite

```bash
ctest -R <name>   # run a single test suite by name
```

| Suite | Tests | What it verifies |
|-------|-------|-----------------|
| `ThermalModel` | 4 | Energy conservation, monotone coolant chain, physical bounds, higher-flow → lower-temperature |
| `PidController` | 6 | Zero-error → min flow, saturation, reset, integrator wind-up |
| `MpcSolver` | 5 | Converges to interior optimum, respects box constraints, warm-start, scalar/vector cases |
| `Integration` | 3 | Full PID + MPC runs produce output CSV, no exceptions |
| `ConfigValidation` | 6 | YAML load, missing keys, out-of-range values, cross-field checks |

**Non-negotiable invariants**: if `ThermalModel.EnergyConservation` or `MpcSolver.ConvergesToInteriorMinimum` fails after a model or solver change, the change is wrong — not the test.

---

## Configuration

All physical parameters and tuning live in `config/`. Three scenario files:

| File | Scenario | Duration |
|------|----------|----------|
| `default.yaml` | Constant 5C discharge | 600 s |
| `scenario_step_transient.yaml` | 1C → 5C step at t = 120 s | 300 s |
| `scenario_rising_ambient.yaml` | T_inlet 25 → 27 °C ramp | 300 s |

The controller type in the YAML (`pid` or `mpc`) can be overridden on the command line:

```bash
./build/btm config/scenario_step_transient.yaml pid   # override to PID
./build/btm config/scenario_step_transient.yaml mpc   # override to MPC
```

---

## Non-goals (deliberate scope limits)

- Not CFD or 3D flow simulation
- Not a production controller (no safety case, no HIL, no RTOS)
- No electrochemical or aging model
- No hardware
- MPC solver is gradient-descent by design — understandable baseline, not state-of-the-art QP

---

## Repository Structure

```
├── LICENSE                        # MIT
├── compile_commands.json          # symlink → build/; clangd IDE support
├── docs/
│   ├── DESIGN.md                  # authoritative model + controller spec
│   ├── ARCHITECTURE.md            # physical model equations, module breakdown, results
│   └── JUDGMENT.md                # historical technical review
├── config/                        # YAML parameter files (single source of truth)
├── include/btm/                   # public headers
├── src/                           # implementations
├── test/                          # GoogleTest suite
├── scripts/                       # Python comparison + plotting
└── results/                       # gitignored — populated by runs
```
