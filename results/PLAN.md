# Experiment Plan

## Completed (copied to results/)

| City    | Scenario        | Fleet sizes                  | Note |
|---------|-----------------|------------------------------|------|
| Boston  | rtv_direct      | 190, 380, 760, 1518, 3038    | ✓ all 5 |
| Boston  | rtv_integrated  | 190, 380, 760, 1519, 3038    | ✓ all 5; **190 used RV_K=15 — re-run with RV_K=30** |
| Chicago | rtv_integrated  | 178, 356, 713, 1426, 2852    | ✓ all 5 |

---

## TODO

### Build configuration reminder

| Scenario         | `PRUNING_RV_K` | `PRUNING_RR_K` | `LINEAR_ASSIGNMENT` |
|------------------|----------------|----------------|---------------------|
| rtv_direct       | 0              | 0              | false               |
| rtv_integrated   | 30             | 30             | false               |
| lp_direct        | 0              | 0              | true                |
| lp_integrated    | 0              | 0              | true                |

Set these in `headers/settings.hpp` and `make` before running.

---

### 1. Chicago — rtv_direct (ride-share only, ILP)
**Build:** `PRUNING_RV_K=0`, `PRUNING_RR_K=0`, `LINEAR_ASSIGNMENT=false`

Fleet sizes: **178, 356, 713, 1426, 2852**

Based on Boston direct best penalty_c values (0.7–0.8 small fleet, 0.8–1.0 mid, escalating large),
suggested search range per fleet size:

| Fleet | Starting pen_c | Search range |
|-------|---------------|--------------|
| 178   | 0.7           | 0.5 – 1.0   |
| 356   | 0.8           | 0.6 – 1.0   |
| 713   | 0.8           | 0.6 – 1.2   |
| 1426  | 1.0           | 0.8 – 1.5   |
| 2852  | 1.5           | 1.0 – 3.0   |

Result folder naming: `INT_30_RV_0_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_ad_pen_{c}`
Output path: `outputs_new/chicago/direct3/{fleet}/4/`

---

### 2. Boston — rtv_integrated re-run for 190 vehicles
**Build:** `PRUNING_RV_K=30`, `PRUNING_RR_K=30`, `LINEAR_ASSIGNMENT=false`

The current best for 190 vehicles used `RV_K=15`. Re-run with `RV_K=30` to be consistent.

| Fleet | Current best | pen_c to try |
|-------|-------------|--------------|
| 190   | 13.44% (RV_K=15) | 0.4, 0.5, 0.6 |

Result folder naming: `INT_30_RV_30_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_pen_{c}_sl`
Output path: `outputs_new/boston/integrated3/190/4/`

---

### 3. Boston — lp_direct (ride-share only, Linear assignment)
**Build:** `PRUNING_RV_K=0`, `PRUNING_RR_K=0`, `LINEAR_ASSIGNMENT=true`

Fleet sizes: **190, 380, 760, 1519, 3038**  
(190 and 3038 have old runs in `linear1/direct/` but at mismatched sizes — re-run all 5.)

| Fleet | Suggested pen_c range |
|-------|-----------------------|
| 190   | 0.5 – 1.0             |
| 380   | 0.5 – 1.0             |
| 760   | 0.5 – 1.0             |
| 1519  | 0.5 – 1.5             |
| 3038  | 1.0 – 3.0             |

Result folder naming: `INT_30_RV_0_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_ad_pen_{c}`
Output path: `outputs_new/boston/lp_direct/{fleet}/4/`

---

### 4. Boston — lp_integrated (transit-integrated, Linear assignment)
**Build:** `PRUNING_RV_K=0`, `PRUNING_RR_K=0`, `LINEAR_ASSIGNMENT=true`, `ALLOW_MULTI_MODAL=true`, `ONLY_ALLOW_SINGLE_LEG=true`

Fleet sizes: **190, 380, 760, 1519, 3038**

| Fleet | Suggested pen_c range |
|-------|-----------------------|
| 190   | 0.4 – 0.8             |
| 380   | 0.5 – 1.0             |
| 760   | 0.7 – 1.2             |
| 1519  | 1.0 – 1.5             |
| 3038  | 1.5 – 3.0             |

Result folder naming: `INT_30_RV_0_RR_0_limit_5_min_pen_10_GRB_20_RTV_30_pen_{c}_sl`
Output path: `outputs_new/boston/lp_integrated/{fleet}/4/`

---

### 5. Chicago — lp_direct (ride-share only, Linear assignment)
**Build:** `PRUNING_RV_K=0`, `PRUNING_RR_K=0`, `LINEAR_ASSIGNMENT=true`

Fleet sizes: **178, 356, 713, 1426, 2852**

Use same pen_c ranges as Chicago rtv_direct (item 1 above).

Output path: `outputs_new/chicago/lp_direct/{fleet}/4/`

---

### 6. Chicago — lp_integrated (transit-integrated, Linear assignment)
**Build:** `PRUNING_RV_K=0`, `PRUNING_RR_K=0`, `LINEAR_ASSIGNMENT=true`, `ALLOW_MULTI_MODAL=true`, `ONLY_ALLOW_SINGLE_LEG=true`

Fleet sizes: **178, 356, 713, 1426, 2852**

Use Chicago rtv_integrated best pen_c values as starting points: 0.4, 0.5, 0.9, 1.5, 2.0

Output path: `outputs_new/chicago/lp_integrated/{fleet}/4/`

---

## Priority order

1. Chicago rtv_direct (unblocks the Chicago comparison)
2. Boston rtv_integrated re-run at 190 (small fix)
3. Boston lp_direct + lp_integrated (completes Boston)
4. Chicago lp_direct + lp_integrated (completes Chicago)
