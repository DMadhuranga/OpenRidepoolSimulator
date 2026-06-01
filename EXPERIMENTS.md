# Experiments

This document describes the full experimental setup for the paper
"Integrating On-demand Ride-sharing with Mass Transit at-Scale" (arXiv 2404.07691).

---

## Research question

We compare four fleet-assignment strategies across five U.S. cities at five fleet-size levels:

| Scenario label  | Assignment solver  | Transit integration |
|-----------------|--------------------|---------------------|
| `rtv_direct`    | ILP (RTV graph)    | No — ride-share only |
| `rtv_integrated`| ILP (RTV graph)    | Yes — first/last-mile + transit |
| `lp_direct`     | LP relaxation      | No — ride-share only |
| `lp_integrated` | LP relaxation      | Yes — first/last-mile + transit |

---

## Cities and fleet sizes

Five U.S. cities. Fleet sizes are approximately `V = round(N_window × C)` where
`N_window` is the number of requests in the simulation window and
`C ∈ {0.1, 0.05, 0.025, 0.0125, 0.00625}` (factors of 2 apart).

| City    | Fleet sizes                    | Data directory               |
|---------|--------------------------------|------------------------------|
| Atlanta | 96, 192, 383, 767, 1533        | `data/atlanta/`              |
| Boston  | 190, 380, 760, 1518/1519, 3038 | `data/boston/`               |
| Chicago | 178, 356, 713, 1426, 2852      | `data/chicago/`              |
| Houston | 115, 230, 459, 918, 1837       | `data/houston/`              |
| LA      | 294, 588, 1175, 2350, 4701     | `data/la/`                   |

All cities use the same simulation window: **6:00 AM – 10:00 AM**
(`INITIAL_TIME=60000`, `FINAL_TIME=100000` in HHMMSS format).

---

## Fixed simulation parameters

These are the same across all scenarios and cities:

```
INTERVAL         30       # seconds between assignment epochs
MAX_WAITING      1200     # max passenger wait time (seconds)
MAX_DETOUR       1.2      # max detour factor over direct travel time
CARSIZE          4        # vehicle capacity
GRB_TIME_LIMIT   20       # Gurobi solver time limit per epoch (seconds)
RTV_TIMELIMIT    30000    # RTV graph time limit per vehicle (milliseconds)
FIX_ASSIGNMENT_BEFORE 300 # lock assignment within 300s of pickup
CTSP             FIX_PREFIX
CTSP_OBJECTIVE   CTSP_VMT
ALLOW_MULTI_MODAL  true   # for integrated scenarios; false for direct
ONLY_ALLOW_SINGLE_LEG true # only first-leg or last-leg (not both)
```

---

## Build configurations

**Compile-time constants** in `headers/settings.hpp` must be set before `make`,
as they are `#define` values not runtime flags:

| Scenario         | `PRUNING_RV_K` | `PRUNING_RR_K` |
|------------------|----------------|----------------|
| `rtv_integrated` | **30**         | **30**         |
| `rtv_direct`     | 0              | 0              |
| `lp_direct`      | 0              | 0              |
| `lp_integrated`  | 0              | 0              |

Change the values in `headers/settings.hpp`, then run `make` before launching runs.

---

## DEMAND_PENALTY_C tuning

`DEMAND_PENALTY_C` (runtime flag `DEMAND_PENALTY_C`) scales the ILP penalty for
unserved requests: `miss_penalty = C × avg_travel_duration`. The best value is
found empirically per (city, scenario, fleet-size) via binary/linear search over
service rate. Multiple runs with different values are saved to `outputs_new/` and
the best (highest service rate) is selected.

**Observed best values from completed runs:**

| City    | Scenario        | Fleet  | Best pen_c |
|---------|-----------------|--------|------------|
| Boston  | rtv_direct      | 190    | 0.7        |
| Boston  | rtv_direct      | 380    | 0.8        |
| Boston  | rtv_direct      | 760    | 0.8        |
| Boston  | rtv_direct      | 1518   | 1.0        |
| Boston  | rtv_direct      | 3038   | 3200 (anomaly — verify) |
| Boston  | rtv_integrated  | 190    | 0.5 (RV_K=15 — needs re-run at RV_K=30) |
| Boston  | rtv_integrated  | 380    | 0.9        |
| Boston  | rtv_integrated  | 760    | 1.0        |
| Boston  | rtv_integrated  | 1519   | 1.3        |
| Boston  | rtv_integrated  | 3038   | 2.5        |
| Chicago | rtv_integrated  | 178    | 0.4        |
| Chicago | rtv_integrated  | 356    | 0.5        |
| Chicago | rtv_integrated  | 713    | 0.9        |
| Chicago | rtv_integrated  | 1426   | 1.5        |
| Chicago | rtv_integrated  | 2852   | 2.0        |

---

## Run command template

```bash
./prog <num_threads> \
  DATAROOT data/<city> \
  RESULTS_DIRECTORY outputs_new/<city>/<scenario_folder>/<fleet>/4/<result_folder> \
  VEHICLE_LIMIT <fleet> \
  CARSIZE 4 \
  INTERVAL 30 \
  MAX_WAITING 1200 \
  MAX_DETOUR 1.2 \
  GRB_TIME_LIMIT 20 \
  RTV_TIMELIMIT 30000 \
  DEMAND_PENALTY_C <pen_c> \
  ALLOW_MULTI_MODAL <true|false> \
  ONLY_ALLOW_SINGLE_LEG true \
  INITIAL_TIME 60000 \
  FINAL_TIME 100000 \
  LINEAR_ASSIGNMENT <true|false>
```

### Result folder naming conventions

| Scenario         | `<scenario_folder>` in path | `<result_folder>` name |
|------------------|-----------------------------|------------------------------------|
| `rtv_integrated` | `integrated3`               | `INT_30_RV_30_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_pen_{c}_sl` |
| `rtv_direct`     | `direct3`                   | `INT_30_RV_0_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_ad_pen_{c}` |
| `lp_integrated`  | `lp_integrated`             | `INT_30_RV_0_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_pen_{c}_sl` |
| `lp_direct`      | `lp_direct`                 | `INT_30_RV_0_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_ad_pen_{c}` |

The `/4/` path component is the `CARSIZE` value.

---

## Output files

Each result folder contains:

| File                   | Contents |
|------------------------|----------|
| `results.log`          | Per-epoch stats + `FINAL SUMMARY` with service rate, MM breakdown, compute time |
| `ilp.csv`              | Per-epoch ILP solve metrics (timestamp, obj, solver time, gap, assigned count, status) |
| `actions.log`          | Per-vehicle move/pickup/dropoff events — used by `analyze.py` for VMT and constraint verification |
| `assignment.log`       | Per-epoch vehicle-to-request assignments |
| `rebalance.log`        | Per-epoch rebalancing targets |
| `unassigned_requests.log` | Requests not served each epoch |
| `debug.log`            | Verbose internal log — large (~100–200 MB), skip when copying |

The `FINAL SUMMARY` block in `results.log` reports:
- `Service Rate` — overall % of requests served (direct + multi-modal)
- `On-demand rate`, `Multi-modal rate`, `First-leg rate`, `Last-leg rate`, `Both-leg rate`
- `Compute Time` — total wall-clock seconds

---

## Directory layout

### Raw experiment outputs: `outputs_new/`

```
outputs_new/
  <city>/
    direct3/          ← rtv_direct runs
    integrated3/      ← rtv_integrated runs
    lp_direct/        ← lp_direct runs
    lp_integrated/    ← lp_integrated runs
      <fleet>/
        4/            ← CARSIZE=4
          <result_folder>/   ← one folder per pen_c value tried
```

Multiple `<result_folder>` entries exist per fleet size (one per pen_c value tried
during the search). This folder is intentionally messy — it holds all trial runs.

### Clean best results: `results/`

```
results/
  <city>/
    <scenario>/       ← rtv_direct | rtv_integrated | lp_direct | lp_integrated
      <fleet>/        ← best-performing result files copied here directly
```

Only the single best-performing run (highest service rate) per (city, scenario, fleet)
is kept here. `debug.log` is excluded. This is the directory `analyze.py` should
point to for final analysis.

**To update `results/` after new runs:** run the selection script in `analyze/`
(or re-run the copy logic in this repo's history) which scans `outputs_new/`,
finds the best service rate per combination, and copies the files.

---

## Analysis script: `analyze/analyze.py`

Parses `actions.log` to compute VMT, service rate, and multi-modal breakdown
independently of `results.log` (cross-check). Also reads `ilp.csv` for solver time.

Key function: `get_vmt(result_folder, requests)` — returns:
`service_rate, total_distance, mm_count, first_legs, last_legs, both_legs, ilp_solve_time, compute_time, served_req, served_legs`

`write_stats_to_csv(city, vehicle_sizes)` aggregates results for a city into
`outputs_new/<city>/stats_<scenario>_4.csv`.

`LEG_REQ_START_INDEX` separates leg request IDs from regular request IDs:
- All cities: `900000` (LA: `1000000`)

---

## Current experiment status (as of 2026-06-01)

| City    | rtv_direct | rtv_integrated | lp_direct | lp_integrated |
|---------|------------|----------------|-----------|---------------|
| Boston  | ✓ done     | ✓ done *       | ✗ todo    | ✗ todo        |
| Chicago | ✗ todo     | ✓ done         | ✗ todo    | ✗ todo        |
| Atlanta | —          | —              | —         | —             |
| Houston | —          | —              | —         | —             |
| LA      | —          | —              | —         | —             |

\* Boston `rtv_integrated` fleet=190 used `PRUNING_RV_K=15` instead of 30 — should be re-run.

Completed results are in `results/`. See `results/PLAN.md` for the priority-ordered
TODO list with suggested pen_c search ranges for each missing run.
