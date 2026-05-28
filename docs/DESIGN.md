# DESIGN.md — Battery Thermal MPC

**Authoritative project specification.**  
This document is the single source of truth for the Battery Thermal MPC project. It must be read before any non-trivial design decision or code change. Conflicts between implementation and this document must be raised explicitly.

Predictive thermal management for an immersion-cooled 24s13p lithium-ion battery module. Pluggable controllers (PID baseline, MPC) regulate coolant mass flow rate to satisfy thermal constraints while minimising pump energy.

---

## 1. Context

### 1.1 Origin

This project is grounded in prior experimental work on the NGS BP9 battery module. That prior work produced:

- A 24s13p module using **Molicel INR-21700-P45B** cells, designed for sustained 5C discharge.
- A **direct-immersion thermal management system** using **Shell E5 TM 410** dielectric coolant.
- Concept validation demonstrating peak cell temperatures below 30 °C and ΔT < 5 °C during 5C discharge.
- A black-box lumped-parameter cell model fitted via Levenberg-Marquardt optimisation (mean error 0.2 %, peak error 0.43 % vs experimental cell voltage). This produced a fitted ohmic overpotential at 1C of **η_IR,1C = 0.077837 V**, which is the heat-generation parameter used throughout this project.

That prior work answered: *can immersion cooling keep this module within thermal limits at 5C?* The answer was yes. This project answers the natural follow-up: *given that thermal capability, what is the optimal way to control the pump?*

A fixed-flow or naive bang-bang strategy meets the constraint by over-cooling, which costs parasitic pump energy and reduces usable pack energy. A predictive controller can anticipate downstream heating, react before sensor readings cross thresholds, and trade pump effort against thermal margin in a principled way. That trade-off is the core of this project.

### 1.2 Why MPC, specifically

PID on the hottest cell temperature reacts after the temperature has already started rising. For a serial coolant chain like immersion cooling, the downstream cells (positions near the coolant outlet) always run hotter than upstream cells because they see pre-heated coolant. During a load transient, position 24 will exceed the constraint several seconds before position 1 sensors even notice — by which time PID is already too late.

MPC exploits the model. It rolls the thermal dynamics forward over a horizon, sees the predicted constraint violation, and adjusts the *current* flow command to prevent it. It also handles the multi-objective nature of the problem natively: minimise pump energy *subject to* T_max < 35 °C and ΔT < 5 °C. PID conflates these into a single tracking error and cannot reason about constraints explicitly.

### 1.3 Why this project

This is a portfolio piece for a Planning and Control Software Engineer role. The role asks for C++ for real-time control, MPC and optimal control familiarity, and automotive domain knowledge. This project demonstrates all three in a problem grounded in real prior experimental work, with real measured data, on a real cell datasheet. The cell parameters, coolant properties, and thermal constraints are not invented.

---

## 2. Scope

### 2.1 In scope

- A lumped-capacitance thermal model of the 24-position immersion-cooled module with algebraic coolant chain energy balance.
- Heat generation from ohmic losses only, using the η_IR,1C parameter from the Levenberg-Marquardt cell fit.
- A `Controller` concept with two implementations: PID and MPC.
- A custom gradient-descent MPC solver implemented from scratch (no external optimisation library) operating on a control-input sequence.
- A discrete-time simulation harness that runs a configurable discharge scenario, logs state and control trajectories to CSV, and produces plots.
- Single-source-of-truth YAML configuration loaded and validated at startup. Invalid configurations fail loudly before simulation begins.
- A test suite covering thermal model conservation, controller correctness on trivial cases, solver convergence, configuration validation, and end-to-end constraint satisfaction.
- Three benchmark scenarios: constant 5C discharge, step load transient (1C → 5C), and rising-ambient inlet temperature.
- A comparison harness that runs the same scenario under PID and MPC and quantifies pump-energy savings at equal constraint satisfaction.

### 2.2 Out of scope

- Cell-to-cell thermal conduction (negligible under immersion).
- Multi-physics coupling (no electrochemical state inside the loop; current draw is exogenous).
- 2D or 3D thermal effects within a cell (lumped capacitance is the deliberate simplification).
- Aging, capacity fade, or SOC drift effects on heat generation.
- State estimation; we assume full state observability (all 24 cell temperatures sensed). Kalman filtering is plausible future work.
- Quadratic programming solvers (OSQP, qpOASES). Replacing the gradient-descent solver with a proper QP is documented future work.
- Hardware deployment. This is a desktop simulation. Real-time guarantees are discussed but not enforced via an RTOS.
- BMS interactions (cell balancing, fault handling).

### 2.3 Explicit non-goals

- This is not a CFD project. The thermal model is a lumped-capacitance approximation, deliberately coarse to make MPC tractable in real time. CFD-grade accuracy is neither claimed nor needed for control design — the controller must be robust to model error, which is a feature, not a bug.
- This is not a production codebase. It targets clarity and correctness over performance optimisation. No SIMD intrinsics, no lock-free queues, no custom allocators.

---

## 3. Approach

### 3.1 Physical model

#### 3.1.1 Module topology under immersion cooling

24 series positions, 13 cells in parallel at each position. Under immersion cooling, every cell at a given series position is bathed in the same coolant stream and carries identical current (I_pack / 13). The 13 parallel cells are therefore thermally indistinguishable and collapse to one effective thermal node per series position. The state dimension drops from 312 to 24, with no loss of physical fidelity given the immersion assumption.

Coolant flows along the series direction: it enters at position 1, picks up heat from the 13 cells there, continues to position 2, and so on through position 24. The coolant temperature at any position is determined by upstream heat pickup and current mass flow rate — it is **algebraic**, not a separate dynamic state. This is the key modelling simplification that keeps the state vector at 24.

#### 3.1.2 Heat generation

Ohmic only, per the project's stated simplification. Using the fitted parameter from the Levenberg-Marquardt cell model optimisation:

```
Q_cell = η_IR,1C × I_cell² / I_1C
```

where I_1C = Q_cell,0 / 3600 s = 4.5 A, and I_cell is the per-cell current. At a 5C pack discharge, I_cell = 22.5 A → Q_cell ≈ 8.76 W per cell, or ≈ 114 W per series position, or ≈ 2.73 kW for the full module. These numbers are consistent with prior experimental validation results showing thermal capability at this duty.

The η_IR,1C value already absorbs internal cell resistance, busbar losses at the cell-tab level, and any other ohmic dissipation captured by the parameter fit. Using it directly is more defensible than picking a textbook DC impedance, because it was *measured* on the same cell chemistry under DLC excitation.

#### 3.1.3 Thermal dynamics

**Single-node model (default)**

For each series position i ∈ {1, …, 24}, the lumped energy balance is:

```
C_th × dT_cell,i / dt = Q_pos,i − h_conv × A_wet × (T_cell,i − T_coolant,i)
```

where:

- `C_th` is the lumped thermal capacitance of the 13-cell parallel group at that position (13 × m_cell × c_p,cell).
- `Q_pos,i = 13 × Q_cell` is the total heat generated at the position.
- `h_conv × A_wet` is the convective conductance from the cell surface to the surrounding coolant.
- `T_coolant,i` is the coolant temperature at position i, computed algebraically (see below).

The coolant temperature at position i is given by chain energy balance:

```
T_coolant,i = T_inlet + (1 / (ṁ × c_p,coolant)) × Σ_{j=1..i} h_conv × A_wet × (T_cell,j − T_coolant,j)
```

This is implicit because T_coolant,i depends on the heat flux which depends on T_coolant,i. The equation is resolved with a fixed three iterations of successive substitution, using the upstream coolant temperature from the previous time step for the first iteration. For dt ≤ 0.1 s and the expected mass-flow and heat-generation ranges, the residual after three iterations is guaranteed < 0.02 °C. This bound is enforced and checked in the thermal model unit tests.

**Two-node model (optional, YAML: `cell.model: two_node`)**

The single-node model assumes the entire cell cross-section is isothermal.  Under
high-rate discharge (≥ 3C) the internal temperature gradient between the jellyroll
core and the aluminium-clad surface is significant — physically around 5–10 °C for
cylindrical 21700-format cells.  The two-node model resolves this gradient by adding
a separate **core** capacitance coupled to the **can** (surface) node through an
internal thermal resistance:

```
C_core × dT_core,i / dt =  Q_pos,i − (T_core,i − T_can,i) / R_core_can
C_can  × dT_can,i  / dt = (T_core,i − T_can,i) / R_core_can − h·A·(T_can,i − T_coolant,i)
```

where:

| Symbol | Default | Meaning |
|--------|---------|---------|
| `C_core = (1 − f) × C_th` | f = 0.10 | Core thermal mass (jellyroll, ~90 % of cell mass) |
| `C_can  =  f × C_th`      | f = 0.10 | Can shell thermal mass (~10 %, thin Al wall) |
| `R_core_can` | 0.8 K/W | Core-to-can thermal resistance |

**Steady-state gradient (analytical)**  
At steady state dT/dt = 0 for both nodes:
```
T_core − T_can = Q_cell × R_core_can = 8.76 W × 0.8 K/W ≈ 7.0 °C  (at 5C)
```
Tested range: 4–10 °C (covers numerical integration error and parameter uncertainty).

**Key design choice:** `ThermalState::cell_temperatures` always holds the **can / surface**
temperature — the externally observable quantity.  `ThermalState::core_temperatures` holds
the **core** temperature, which is the safety-critical internal maximum.
Safety constraints (`T_max < 35 °C`) and the MPC cost function both operate on
`max_core_temp()`.  In single-node mode `max_core_temp() == max_cell_temp()`, so all
existing results and CI thresholds are unchanged when `cell.model: single_node`.

The coolant chain still couples to the **can** temperature (the physical contact surface),
so `coolant_temperatures()` is correct for both models.

**YAML parameters** (all optional — default to single-node if `cell.model` absent):
```yaml
cell:
  model: single_node          # "single_node" | "two_node"
  # Two-node parameters (only read when model: two_node):
  # r_core_can_k_per_w: 0.8  # core→can resistance (K/W)
  # c_can_fraction: 0.10      # fraction of C_th in can shell
```

#### 3.1.4 Convective coefficient (two selectable models)

The convective coefficient h [W/(m²·K)] depends on mass flow rate ṁ.  Two models are
implemented and selected via `convection.model` in YAML.

**Power-law (default, backward-compatible)**

```
h(ṁ) = h_ref · (ṁ / ṁ_ref)^n
```

`h_ref = 250 W/(m²·K)` is calibrated to match the experimentally-validated ~30 °C peak at 5C.
`n = 0.6` is the standard forced-convection scaling exponent.  This model is simple, auditable,
and reproduces the headline results exactly.

**Nusselt correlation** (`convection.model: nusselt_correlation`)

```
Re  = ṁ · D_h / (A_flow · μ_coolant)
Pr  = μ_coolant · cp_coolant / k_coolant
Nu  = c · Re^m · Pr^n
h   = Nu · k_coolant / D_h
```

`D_h` (hydraulic diameter) and `A_flow` (effective total cross-sectional flow area) are new
geometric YAML parameters.  At the operating conditions for Shell E5 TM 410 dielectric oil
(ρ = 805 kg/m³, μ = 0.012565 Pa·s, cp = 3500 J/(kg·K), k = 0.13 W/(m·K)):

- **Re ≈ 600** at ṁ = 0.5 kg/s with D_h = 6 mm, A_flow = 400 mm²  → **laminar regime**
- **Pr ≈ 338** (high-Prandtl viscous oil)
- Sieder-Tate laminar parameters: c = 0.197 (calibrated), m = 0.333, n = 0.333
- Result: h(0.5 kg/s) ≈ 250 W/(m²·K), matching the power-law calibration point

The parameters c, m, n are configurable in YAML to allow Dittus-Boelter turbulent (c = 0.023,
m = 0.8, n = 0.4) or any other peer-reviewed correlation as better experimental data becomes
available.  Both models produce monotonically increasing h(ṁ); this invariant is enforced in the
test suite.

### 3.2 Control strategy

#### 3.2.1 Controller interface

A `Controller` C++20 concept defines what the simulator requires. PID and MPC both satisfy it. There is no inheritance hierarchy; the simulator is templated on the controller type. This is composition: the simulator does not know or care which controller it holds, only that it can ask it for a command given a state.

```cpp
template <typename C>
concept Controller = requires(C& c, const ThermalState& s, Duration dt) {
    { c.compute_command(s, dt) } -> std::convertible_to<MassFlowRate>;
    { c.reset() } -> std::same_as<void>;
};
```

The rationale for a concept over an abstract base class: zero virtual call overhead in the simulation loop (the MPC controller will be called at every timestep, hundreds of thousands of times); compile-time substitutability; and no inheritance hierarchy to maintain. The cost is that the simulator must be templated, which means the controller type is fixed at compile time per binary. That is acceptable here — controller selection happens via the config file, dispatched once in `main()`.

#### 3.2.2 PID baseline

Proportional-integral control on the maximum observed cell temperature against a setpoint (default 30 °C). Integral wind-up is bounded by clamping the integrator to a configurable window. Output is mapped from the controller signal to a mass flow rate in [ṁ_min, ṁ_max] via saturation.

PID is included not because it is the right answer, but because the MPC needs something to beat. A working PID also catches integration bugs: if PID cannot stabilise the system, the model is broken.

#### 3.2.3 MPC formulation

Decision variable: the sequence of N control inputs u = [ṁ_0, ṁ_1, …, ṁ_{N-1}] over a finite horizon N (default 20 steps at dt = 0.1 s = 2 s horizon).

Cost function:

```
J(u) = Σ_{k=0..N-1} [
        w_T  × Σ_i (T_cell,i(k) − T_ref)²
      + w_Δ  × (max_i T_cell,i(k) − min_i T_cell,i(k))²
      + w_u  × ṁ(k)²
      + w_du × (ṁ(k) − ṁ(k-1))²
    ]
```

The terms encode, in order: tracking a desired cell temperature, penalising thermal non-uniformity across the module, penalising pump power (which scales with ṁ²·³ for centrifugal pumps; we use ṁ² as a tractable proxy), and penalising flow-rate slew (avoids chattering and respects pump dynamics).

**Constraints (T6 — core-temperature-aware):**

- **Hard input bounds:** ṁ_min ≤ ṁ(k) ≤ ṁ_max for all k. Enforced by projecting each iterate onto the feasible interval after every gradient step.
- **Soft core constraint:** T_core,i < T_core_max (default 35 °C). Enforced via quadratic penalty `P_core × max(0, T_core,i − T_core_max)²`. In single-node mode, T_core = T_cell — backward-compatible with all prior results.
- **Soft can constraint (T6, optional):** T_can,i < T_can_max. Penalty weight `soft_T_can_penalty` defaults to 0 (disabled). Enables a secondary surface-temperature guard when using two-node model.
- **Soft ΔT constraint:** max_i T_can,i − min_i T_can,i < ΔT_max (default 5 °C). The inter-cell spread is computed on CAN temperatures (externally observable).

Named constraint fields in `thermal_constraints` YAML section:
```yaml
thermal_constraints:
  max_cell_temperature_c: 35.0       # backward-compat primary limit
  max_temperature_delta_c: 5.0       # inter-cell ΔT
  # max_core_temperature_c: 35.0    # T6: core safety limit (defaults to max_cell)
  # max_can_temperature_c: 35.0     # T6: surface limit (defaults to max_cell)
```

`SimResult` now tracks `violation_core_count`, `violation_can_count`, and `violation_core_time_s`, `violation_can_time_s` separately, enabling per-type constraint auditing.

This is a soft-constrained, input-bound-constrained nonlinear MPC. The nonlinearity comes from the h_conv(ṁ) dependence; the rest of the dynamics are linear in state.

#### 3.2.4 Solver: gradient descent on the input sequence

A single-shooting approach:

1. Initialise u from the previous solution shifted by one step (warm start), with the last entry copied from the second-to-last. On the first call, initialise with a flat profile at mid-range flow.
2. Roll the model forward N steps under u, recording states.
3. Compute J.
4. Compute ∂J/∂u_k for each k by finite differences (perturb u_k by ε, re-roll, measure ΔJ). This costs N forward rollouts per gradient evaluation — acceptable for N = 20.
5. Project-gradient step: u_k ← clip(u_k − α × ∂J/∂u_k, ṁ_min, ṁ_max).
6. Repeat until ‖ΔJ‖ < tol or iteration cap.
7. Return u_0 as the control command for this step.

Why finite-difference gradients instead of analytical/adjoint? The model is small (24 states, short horizon, simple dynamics), so the cost is manageable. Analytical adjoints would be faster but require deriving and maintaining the adjoint equations alongside the forward model, which is more code to test for a marginal speedup on this problem size. This is a deliberate engineering trade-off, documented as such, with adjoint methods listed as future work.

The step size α is fixed (line search is overkill for the first implementation, and the cost surface is reasonably well-conditioned for this problem). Convergence is monitored on relative change in J; the iteration cap is a hard safety bound to keep the per-timestep solve time predictable.

This solver is not state-of-the-art. It is *understandable*, deterministic, and provides a clear baseline against which a future QP-based MPC can be compared.

### 3.3 Validation scenarios

Three scenarios exercise the controller along distinct axes:

**Constant 5C discharge.** The nominal design duty cycle. Both PID and MPC should keep T_max < 35 °C and ΔT < 5 °C. The metric is *integrated pump effort* (∫ ṁ² dt) over a 600 s discharge. MPC should win by a measurable margin because it can run leaner near steady state.

**Step load transient.** Discharge steps from 1C to 5C at t = 100 s. This is where MPC visibly beats PID. The PID controller will only react after the cell temperature begins rising, and given the thermal inertia of the cells plus the coolant transport delay, will briefly overshoot the 35 °C limit. MPC sees the current step only through the instantaneous heat generation term at each time step (the controller receives no preview of the future discharge current) and ramps flow proactively. The headline success criterion (MPC keeps T_max < 35 °C while PID briefly violates it) is defined and measured under this no-preview policy. The scenario harness records the preview flag (always false for the headline comparison) so that any future reproduction is exact.

**Rising ambient inlet temperature.** T_inlet rises from 20 °C to 30 °C linearly over the run. Tests robustness to disturbance. Both controllers should adapt, but a fixed-gain PID will degrade more than an MPC that retunes its action automatically against the current state.

Each scenario runs both controllers and emits per-step CSV: time, all 24 cell temperatures, all 24 coolant temperatures, mass flow command, instantaneous heat generation, constraint flags. Plots are generated by Python post-processing scripts.

### 3.4 MPC horizon selection methodology (T3)

#### Thermal time constant

The dominant time constant of a single lumped cell is:

```
τ = C_th / (h·A) = (m_cell·cp_cell) / (h·A)
  = (0.070 kg × 900 J/(kg·K)) / (250 W/(m²·K) × 0.00475 m²)
  = 63 J/K / 1.1875 W/K
  ≈ 53 s
```

Full settling would require a horizon of N = τ / dt = 530 steps, which is computationally
intractable. However, full settling is not needed — the MPC only needs to *anticipate* the
constraint violation, not to plan all the way to steady state.

#### Practical horizon bound

Two physical delays dominate the MPC's need for preview:

1. **Coolant transport delay**: at nominal flow ṁ = 0.5 kg/s, the coolant traverses 24 positions in
   roughly 0.5–2 s (depending on flow area and channel length) — covered by N ≥ 5–20 steps.
2. **Cell thermal inertia**: the cell takes τ ≈ 53 s to fully respond to a step in Q. However,
   the temperature exceeds the constraint in roughly 10–30 s after a load step, not 53 s.
   N = 20–40 steps (2–4 s) provides sufficient advance notice.

#### Sensitivity analysis

The script `scripts/horizon_analysis.py` sweeps N ∈ {5, 10, 20, 40, 60, 80} on all three
scenarios and reports peak T_max, violations, ∫ṁ² dt, and wall-clock time per MPC step.
A plot is written to `docs/images/horizon_analysis.png`.

**Recommended choice**: **N = 20** (2 s preview at dt = 0.1 s).

Rationale:
- Covers the coolant transport delay (≈ 1–2 s) with margin.
- Provides sufficient preview of the 1C→5C load step to pre-ramp flow before the
  constraint is violated (demonstrated in the step-transient scenario: 0 violations).
- O(N) finite-difference gradient cost stays below 2 ms per MPC step on modern hardware.
- Diminishing returns beyond N = 40 (verified by the sensitivity sweep): pump energy savings
  plateau and solver wall time grows linearly while constraint satisfaction does not improve.

The horizon is a first-class YAML parameter (`mpc.horizon_steps`) and is commented in all
config files with a reference to this section.

### 3.5 Formal metrics definition (T8)

All metrics are computed over the true physical state, independent of sensor mode.

| Metric | Symbol | Definition |
|---|---|---|
| Peak core temperature | T_core_peak | max over all steps of max_i(T_core[i]) — safety-critical internal maximum |
| Time-averaged max core temperature | T_core_avg | (1/N) Σ max_i(T_core[i]) |
| Peak inter-cell ΔT | ΔT_peak | max over all steps of (max_i – min_i)(T_can[i]) — observable surface spread |
| **Core violation count** | N_viol_core | Number of timesteps where T_core_max > max_core_temperature_c |
| **Can violation count** | N_viol_can | Number of timesteps where T_can_max > max_can_temperature_c |
| **Combined violation count** | N_viol | Timesteps where any constraint (core or ΔT) is exceeded |
| **Violation time** | t_viol | N_viol × dt [seconds] |
| **Violation temperature integral** | ∫(T-T_lim)⁺ dt | Σ max(0, T_core_max – T_core_constraint) × dt [°C·s] |
| **Pump control effort** | ∫ṁ² dt | Σ ṁ² × dt [(kg/s)²·s] — proportional to pump energy for centrifugal pump |

**Regression guarantees (enforced by CI):**
- MPC must have zero constraint violations on the default constant-5C scenario.
- MPC pump integral must be at least 30% lower than PID on the same scenario.
- PID peak temperature must remain in the physically plausible range 30–40 °C.

All metrics are written to `<log_path>_summary.json` alongside the CSV for machine-readable downstream consumption (report generator, CI regression guard).

---

## 4. Architecture

### 4.1 Principles

The architecture follows a small set of principles, applied consistently:

- **Pure model, stateful controllers.** The thermal model is a pure function: given (state, input, dt) it returns next state. It owns no state internally. This lets MPC roll the same model forward without contaminating the live simulation state. Controllers, in contrast, own their internal state (integrators, warm-start buffers).
- **Single source of truth for configuration.** All physical parameters, controller gains, scenario definitions, and simulation settings live in one YAML file. There are no hardcoded numbers in source, no environment variables, no scattered defaults. The config is parsed once at startup, validated, and converted into strongly-typed structs that are passed by const reference to whatever needs them.
- **Fail loud at the boundary, never inside the loop.** Configuration is validated *before* the simulator starts. Invalid YAML, out-of-range parameters, unit mismatches, or geometrically impossible setups throw at load time with a clear message naming the offending field. The simulation loop itself contains no defensive coercion; if a precondition is violated mid-loop, that is a logic bug and should crash.
- **Strong types over raw doubles.** Temperature, MassFlowRate, Duration, Current, Power are distinct types (thin wrappers around double with `explicit` constructors and arithmetic operators). The compiler then catches unit errors that would otherwise be silent.
- **Composition over inheritance.** The Controller concept is the only interface that crosses module boundaries between control and simulation. There are no virtual functions in the hot path.

### 4.2 Module breakdown

The project is organised into seven internal modules. Each module owns its own header(s), its own tests, and its own responsibility.

**`core`** — Foundational types and utilities. Strong typedefs (Temperature, MassFlowRate, etc.), simple result/optional types for error handling at the configuration boundary, and shared constants (gas constant, etc.). No dependencies on any other module. This is the bottom of the stack.

**`config`** — YAML loading and validation. Defines the top-level `Config` struct and nested parameter structs (CellParams, CoolantParams, ModuleGeometry, ThermalConstraints, PumpLimits, MpcConfig, PidConfig, ScenarioConfig, SimConfig). Provides `Config load_and_validate(const std::string& path)` which either returns a valid Config or throws with a descriptive error. Depends on `core` and a YAML parser (yaml-cpp).

**`model`** — The thermal model. Implements the lumped 24-position dynamics described in §3.1. The model is a pure function on `(ThermalState, MassFlowRate, Current, Duration) → ThermalState`. It also exposes a coolant-temperature query for visualisation. Constructed once from a validated `Config`; references its parameters by const reference. Depends on `core` and `config`.

**`control`** — Controller implementations. Defines the `Controller` concept, plus `PidController` and `MpcController` types satisfying it. The MPC controller holds a const reference to the model (it uses the same model for its internal rollouts) and owns a `GradientDescentSolver`. Depends on `core`, `config`, `model`, and `solver`.

**`solver`** — The gradient-descent MPC solver, factored out so it can be swapped or replaced (future work: OSQP-based QP solver). Defines a small `Problem` struct (cost function, constraints, gradient callback) and a `Solution` struct (optimal input sequence, final cost, iteration count, convergence flag). Pure algorithmic code; depends only on `core`.

**`scenario`** — Discharge profiles and ambient profiles. A `Scenario` provides `Current current_at(Time t)` and `Temperature inlet_at(Time t)`. Three implementations cover the validation scenarios in §3.3. Depends on `core` and `config`.

**`sim`** — The time-stepping simulation harness and CSV logger. Templated on Controller. Drives the loop, queries the scenario for exogenous inputs, calls the controller, advances the model, logs each step. The logger is a separate type with its own file lifetime; it owns its CSV stream and flushes on destruction. Depends on everything below.

**`main`** — The entry point. Parses command-line arguments (just a config path), loads and validates the config, dispatches on controller type, runs the simulator, and exits. This is the only place where polymorphism over controller type exists, and it is resolved at startup, not in the loop.

The dependency graph is a DAG with `core` at the root and `main` at the leaf. No cycles, no implicit globals, no singletons.

### 4.3 Repository layout

```
battery-thermal-mpc/
├── README.md                      # human entry point and quickstart
├── LICENSE
├── CMakeLists.txt
├── .clang-format
├── .clang-tidy
├── .gitignore
├── config/
│   ├── default.yaml               # full parameter set, single source of truth
│   ├── scenario_constant_5c.yaml
│   ├── scenario_step_transient.yaml
│   └── scenario_rising_ambient.yaml
├── include/btm/
│   ├── core/
│   │   ├── types.hpp              # strong types
│   │   └── constants.hpp
│   ├── config/
│   │   ├── config.hpp             # all parameter structs
│   │   └── loader.hpp
│   ├── model/
│   │   ├── thermal_state.hpp
│   │   └── thermal_model.hpp
│   ├── control/
│   │   ├── controller_concept.hpp
│   │   ├── pid_controller.hpp
│   │   └── mpc_controller.hpp
│   ├── solver/
│   │   └── gradient_descent.hpp
│   ├── scenario/
│   │   └── scenario.hpp
│   └── sim/
│       ├── simulator.hpp
│       └── csv_logger.hpp
├── src/
│   ├── config/loader.cpp
│   ├── model/thermal_model.cpp
│   ├── control/pid_controller.cpp
│   ├── control/mpc_controller.cpp
│   ├── solver/gradient_descent.cpp
│   ├── scenario/scenario.cpp
│   ├── sim/csv_logger.cpp
│   └── main.cpp
├── test/
│   ├── CMakeLists.txt
│   ├── test_core_types.cpp
│   ├── test_config_validation.cpp
│   ├── test_thermal_model.cpp
│   ├── test_pid_controller.cpp
│   ├── test_mpc_solver.cpp
│   ├── test_mpc_controller.cpp
│   ├── test_scenario.cpp
│   └── test_integration.cpp
├── scripts/
│   ├── plot_results.py
│   ├── compare_controllers.py
│   └── requirements.txt
├── results/                       # gitignored, populated by runs
└── docs/
    ├── DESIGN.md                  # authoritative project specification (single source of truth)
    ├── ARCHITECTURE.md            # goals, implementation strategy, phase breakdown, diagrams
    ├── JUDGMENT.md                # senior technical review (historical)
    └── images/                    # generated diagrams and result plots
```

### 4.4 Data flow

A single simulation run executes the following sequence.

1. **Load and validate config.** `main()` calls `config::load_and_validate(path)`. This parses the YAML, populates strongly-typed structs, runs cross-field validation (e.g. ṁ_min < ṁ_max, horizon > 0, series_count > 0, max_cell_temperature_c > inlet_temperature_c), and either returns a valid `Config` or throws.
2. **Construct components.** The thermal model, the chosen controller, the scenario, the logger, and the simulator are constructed from the config. None of them allocate dynamically after construction except the MPC solver's working buffers, which are sized once.
3. **Run loop.** For each timestep:
   - Scenario provides `I_cell(t)` and `T_inlet(t)`.
   - Controller receives the current `ThermalState` and returns a `MassFlowRate` command.
   - Model advances: `state = model.step(state, command, I_cell, dt)`.
   - Logger writes the row.
4. **Finalise.** Logger flushes. Summary statistics (peak T, peak ΔT, integrated pump effort, constraint violations) are printed to stdout. Exit code reflects whether thermal constraints were satisfied — useful for CI.

### 4.5 Configuration: the single source of truth

A representative `default.yaml`:

```yaml
cell:                              # Molicel INR-21700-P45B (datasheet + L-M fit)
  capacity_ah: 4.5
  nominal_voltage_v: 3.6
  mass_kg: 0.070
  specific_heat_j_per_kg_k: 900    # typical Li-ion cylindrical
  surface_area_m2: 0.00475         # 2πrh + 2πr²
  eta_ir_1c_v: 0.077837            # from Levenberg-Marquardt cell model fit
  model: single_node               # "single_node" (default) | "two_node" (DESIGN.md §3.1.3)
  # Two-node parameters (only read when model: two_node):
  # r_core_can_k_per_w: 0.8       # core→can resistance; ΔT_core_can ≈ 7 °C at 5C
  # c_can_fraction: 0.10           # fraction of C_th in can shell

module:
  series_count: 24
  parallel_count: 13

coolant:                           # Shell E5 TM 410
  density_kg_per_m3: 805
  specific_heat_j_per_kg_k: 3500
  dynamic_viscosity_pa_s: 0.012565
  thermal_conductivity_w_per_m_k: 0.13   # typical dielectric coolant
  inlet_temperature_c: 25

convection:
  model: power_law                 # "power_law" (default) | "nusselt_correlation" (§3.1.4)
  h_ref_w_per_m2_k: 250            # at reference flow rate
  m_dot_ref_kg_per_s: 0.5
  scaling_exponent: 0.6            # h ~ (ṁ/ṁ_ref)^0.6

thermal_constraints:
  max_cell_temperature_c: 35.0    # backward-compat primary limit (also default for core/can)
  max_temperature_delta_c: 5.0
  # max_core_temperature_c: 35.0  # T6: internal core limit (defaults to max_cell_temperature_c)
  # max_can_temperature_c: 35.0   # T6: surface limit (defaults to max_cell_temperature_c)

pump:
  min_flow_kg_per_s: 0.01
  max_flow_kg_per_s: 2.0

simulation:
  duration_s: 600
  timestep_s: 0.1
  log_path: "results/run.csv"

scenario:
  type: constant_c_rate            # | step_transient | rising_ambient
  c_rate: 5.0

sensor:
  mode: perfect                    # "perfect" (default) | "downstream" | "sparse" (T1)
  # positions: [7, 15, 23]         # sparse mode only

controller:
  type: mpc                        # | pid
  pid:
    kp: 0.05
    ki: 0.001
    setpoint_c: 30.0
    integrator_limit: 50.0
    deadband_c: 0.0                # |error| < deadband_c treated as zero (T4); 0.0 = disabled
  mpc:
    horizon_steps: 20              # DESIGN.md §3.4: N=20 (2s preview) is the principled choice
    setpoint_c: 34.9
    soft_T_max_penalty: 10000.0   # quadratic penalty (J/°C²) for core constraint violation
    soft_T_can_penalty: 0.0       # T6: can-surface penalty (0.0 = disabled in single-node mode)
    soft_dT_penalty: 10000.0
    weights:
      tracking: 1.0
      delta_t: 10.0
      pump_energy: 0.1
      input_rate: 1.0
    solver:
      max_iterations: 50
      step_size: 0.001
      convergence_tol: 1e-4
      finite_diff_epsilon: 1e-4
```

Validation rules enforced at load:

- All counts (series, parallel, horizon) are positive integers.
- All physical quantities are positive and within physically reasonable ranges (e.g. cell mass < 1 kg, capacity < 100 Ah).
- min < max where applicable (pump bounds, temperature constraints vs inlet temperature).
- Controller type is one of the known values.
- File paths are writable (the log directory exists or can be created).
- Scenario parameters are consistent with the chosen scenario type.

A failure at any of these prints the offending field name, the violating value, and the expected condition. The process exits non-zero. This is the only error-handling layer; once construction succeeds, the loop assumes correctness.

### 4.6 Testing strategy

Tests are organised by module and by level.

**Unit tests** verify individual components in isolation:

- `test_core_types` checks that strong types prevent unit mistakes at compile time (via static_assert on disallowed conversions) and that arithmetic behaves correctly.
- `test_config_validation` feeds malformed configs (negative mass, swapped bounds, unknown controller type, missing fields) and asserts that load_and_validate throws with the expected error keyword.
- `test_thermal_model` verifies energy conservation (total heat in over a step equals stored thermal energy change plus heat removed by coolant, within numerical tolerance), checks the steady-state solution against analytical predictions for simple cases (zero heat generation → all temperatures decay to inlet), and verifies that increasing flow reduces steady-state cell temperatures monotonically. **T7 enhanced validation suite** adds seven additional physics tests for the two-node model and convection alternatives:
  - *TwoNode_CoreHotterThanCan*: after warm-up at 5C, T_core > T_can at all 24 positions.
  - *SingleNode_CoreEqualsCan*: in single-node mode, max_core_temp() == max_cell_temp() at every step (backward-compat invariant).
  - *TwoNode_SteadyStateGradientAt5C*: after 300 s (≈ 3.1 × τ_slow), ΔT_core_can ∈ [5, 9] °C; analytical prediction Q × R = 8.76 × 0.8 ≈ 7 °C.
  - *TwoNode_EnergyConservation*: Q_gen = ΔE_core + ΔE_can + Q_removed, within 5 J/step.
  - *TwoNode_HigherFlowLowerBothTemps*: increasing ṁ from 0.1 to 1.0 kg/s strictly decreases both T_can and T_core at steady state.
  - *TwoNode_ZeroLoadNoGradient*: at zero current, T_core → T_can → T_inlet (no gradient).
  - *TwoNode_CoreLeadsCan_OnLoadStep*: on a 1C→5C step, T_core rises faster than T_can (physical evidence for the two-node model's relevance).
- `test_pid_controller` checks setpoint tracking on a first-order plant proxy, integrator bounding, and saturation behaviour at flow limits.
- `test_mpc_solver` checks convergence on a trivial cost surface (quadratic in u with known minimum), warm-start usage, and respect for input bounds via projection.

**Integration tests** verify components working together:

- `test_integration` runs full short simulations (60 s) under both controllers and asserts that constraint flags are zero and that the MPC achieves lower integrated pump effort than PID on the constant-5C scenario.

A test passes only if both compiled assertions and the runtime exit code are clean. The test binary is invoked by CTest, which CMake configures.

### 4.7 What this architecture is *not* designed for

- **Live tuning.** Parameters are not hot-reloadable. Changing the config requires a rerun. This is intentional; controllers that can have their gains changed mid-run are harder to reason about and harder to test.
- **Multi-threading.** The simulator runs single-threaded. The MPC solver could be parallelised across finite-difference gradient evaluations, but isn't, because (a) the problem size makes it unnecessary and (b) adding threading would complicate testing without changing the architectural story this project is meant to demonstrate.
- **Plugin loading.** Controllers are compile-time selectable, not runtime-loadable. A `dlopen`-based plugin system was rejected as over-engineering for a single-binary simulator.
- **Networked telemetry.** All output is CSV to local disk. There is no socket interface, no shared-memory ringbuffer, no protobuf encoding. Production embedded systems would have these; this is not one.

---

## 5. Timeline

A one-week build budget, roughly 70 hours across seven days. The order minimises risk by getting an end-to-end runnable pipeline early, even if individual components are skeletal at first.

**Day 1 — Foundations.** Repo scaffolding, CMake, .clang-format, .clang-tidy, dependencies (yaml-cpp, GoogleTest). Strong types and constants. Configuration structs and YAML loader with validation. Tests for config loading. End of day: a binary that loads a config and prints it.

**Day 2 — Thermal model.** Implement the 24-position lumped model, including the chain coolant balance. Write the model conservation tests. End of day: model passes energy-conservation tests and produces sensible steady-state temperatures for a hand-checked scenario.

**Day 3 — Simulation harness and PID.** CSV logger, scenario types (constant 5C only for now), the templated simulator, and the PID controller. End of day: full end-to-end run on constant 5C with PID, producing a CSV that plots correctly in Python.

**Day 4 — MPC solver.** Gradient-descent solver as a standalone component with its own tests on a synthetic quadratic problem. End of day: solver converges to known optima on test problems with warm-starting verified.

**Day 5 — MPC controller.** Integrate the solver with the thermal model: rollouts, cost evaluation, finite-difference gradients, warm-start across timesteps. End of day: MPC controller runs end-to-end on the constant 5C scenario, satisfies constraints, and beats PID on integrated pump effort.

**Day 6 — Remaining scenarios and integration tests.** Step-transient and rising-ambient scenarios. Comparison harness. Tune MPC weights. End of day: three scenarios all produce clean plots; MPC visibly beats PID on the step-transient (the headline figure).

**Day 7 — Documentation, polish, push.** README with results, architecture diagram in Mermaid, sample plots embedded. Clean up TODOs and warnings. Push to GitHub with tagged release.

Items deliberately deferred to a "future work" section in the README:

- QP solver replacement (OSQP integration).
- Adjoint-method gradients for the MPC solver.
- State estimation via Kalman filtering (currently assuming full observability).
- Cell aging effects on heat generation.
- Real-time profiling and per-step latency analysis.
- Hardware-in-the-loop interface.

---

## 6. Risks and unknowns

A few things may not go as planned.

**MPC solver may not converge in real time.** The 24-state model with N=20 horizon and finite-difference gradients requires N forward rollouts per gradient evaluation, times the iteration cap. If per-step solve time exceeds the 100 ms simulation timestep, the controller is not real-time. Mitigation: reduce horizon, increase step size, accept partial convergence (stop after k iterations regardless of tolerance). Documenting solve-time statistics is part of the deliverable.

**Convective coefficient calibration uncertainty.** The h_conv reference value is the most uncertain physical parameter. The fallback is to pick a value that reproduces the experimentally-validated peak temperature at the nominal 5C scenario, and treat it as an empirical model parameter. This is honest engineering — the controller's job is to be robust to such uncertainty, not to depend on it being perfectly known.

**MPC tuning weights are non-trivial.** w_T, w_Δ, w_u, w_du form a four-dimensional tuning space. Bad weights yield a controller that either chatters, ignores the temperature constraint, or runs the pump flat-out. The mitigation is incremental tuning: start with only w_T nonzero, get tracking working, then add w_u for energy, then w_Δ for uniformity, then w_du for slew. Document the tuning sequence in the README; it is a substantive piece of the engineering story.

**Model error vs CFD ground truth is unmeasured.** Without CFD data to compare against, there is no quantitative model-error bound. The README must be explicit about this: the model is a control-design model, not a high-fidelity predictor. The controller's job is to be robust to model error.

---

## 7. Success criteria

The project is complete when:

- All three scenarios run end-to-end under both controllers and produce CSVs without errors.
- All unit and integration tests pass.
- On the step-transient scenario, the MPC controller satisfies T_max < 35 °C while PID briefly violates it. This is the headline figure.
- On the constant-5C scenario, MPC achieves at least 10 % lower integrated pump effort than PID at equal constraint satisfaction. (10 % is a defensible target; the actual margin will depend on tuning.)
- The README explains the problem, the architecture, the results, and the limitations clearly enough that a control-software hiring manager reading it for five minutes understands what was built, why, and what it would take to extend it.
- The code passes clang-tidy with the project's `.clang-tidy` ruleset and clang-format on commit.
