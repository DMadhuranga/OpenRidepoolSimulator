# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

Requires Gurobi (set `GUROBI_HOME`) and Boost libraries.

```bash
make          # Build the binary ./prog
make clean    # Remove build artifacts
```

The build compiles all `src/*.cpp`, `src/algorithms/*.cpp`, and `cnpy/cnpy.cpp` into `build/`, links against Gurobi (`libgurobi_c++`, `libgurobi110`) and zlib. No test suite exists.

`mip.cpp` in the root is a standalone Gurobi verification test, not part of the main simulation.

## Running the Simulator

```bash
./prog <num_threads> DATAROOT <path> RESULTS_DIRECTORY <path> [key value ...]
```

Example with common settings:
```bash
./prog 4 DATAROOT data/chicago RESULTS_DIRECTORY results \
  INTERVAL 30 MAX_WAITING 1200 MAX_DETOUR 1.2 CARSIZE 4 \
  INITIAL_TIME 60000 FINAL_TIME 190000 \
  GRB_TIME_LIMIT 20 RTV_TIMELIMIT 10000 \
  ALLOW_MULTI_MODAL false DISABLE_REASSIGNMENT false
```

Key runtime parameters (all have defaults in `src/settings.cpp`):
- `DATAROOT` — root directory for input data (default: `"data"`)
- `RESULTS_DIRECTORY` — output directory (default: `"results"`)
- `INTERVAL` — seconds between assignment epochs (default: 60)
- `MAX_WAITING` — max passenger wait time in seconds (default: 300)
- `MAX_DETOUR` — max detour as factor of direct travel time (default: 600)
- `CARSIZE` — max passengers per vehicle (default: 4); if negative, per-vehicle capacity from `vehicles.csv` is used
- `VEHICLE_LIMIT` — cap on vehicles loaded (default: 1000)
- `INITIAL_TIME` / `FINAL_TIME` — simulation window in HHMMSS format
- `GRB_TIME_LIMIT` — Gurobi solver time limit in seconds
- `RTV_TIMELIMIT` — milliseconds per vehicle for RTV graph generation
- `ALLOW_MULTI_MODAL` — enable multi-modal trips (first/last-mile + transit)
- `LINEAR_ASSIGNMENT` — use LP relaxation instead of full ILP
- `PRE_SOLVE_ILP` — warm-start ILP with LP solution
- `DISABLE_REASSIGNMENT` — prevent re-assigning vehicles with committed routes
- `DISABLE_DIRECT_TRIPS` — disallow single-passenger (non-shared) trips
- `CTSP` — route planner variant: `FULL`, `FIX_ONBOARD`, `FIX_PREFIX` (default), `MEGA_TSP`

## Data Layout

Each city dataset lives under `data/<city>/`:
```
data/<city>/map/nodes.csv
data/<city>/map/edges.csv
data/<city>/map/times.csv      # legacy CSV format
data/<city>/map/times.npy      # preferred: uint16 2D travel-time matrix
data/<city>/vehicles/vehicles.csv
data/<city>/requests/requests.csv
data/<city>/requests/leg_requests.csv  # multi-modal legs
```

Available cities: `atlanta`, `boston`, `chicago`, `houston`, `la`.

The `Network` class loads `times.npy` (a uint16 NumPy array) as the primary travel-time matrix via the `cnpy` library.

### Input File Formats

**`vehicles.csv`**: `driver_id, starting_node, latitude, longitude, start_time, capacity`
```
1,4088,41.8186,-87.6478,00:00:00,4
```

**`requests.csv`**: `request_id, origin_node, origin_lon, origin_lat, dest_node, dest_lon, dest_lat, requested_time`
```
4,6266,-71.0246,42.3989,50429,-70.9847,42.4507,06:00:00
```
`latest_boarding = entry_time + MAX_WAITING`; `latest_alighting = entry_time + MAX_WAITING + MAX_DETOUR * ideal_traveltime`.

**`leg_requests.csv`**: `leg_re_id, origin, origin_lon, origin_lat, dest, dest_lon, dest_lat, earliest_pickup_time, latest_dropoff_time, original_trip_request_id, bus_trip_id, leg_type, info`

`leg_type`: `0` = first leg (origin → transit stop), `1` = last leg (transit stop → destination). `latest_dropoff_time` maps directly to `latest_alighting` (hard bus connection deadline); `latest_boarding` is back-calculated as `latest_alighting - ideal_traveltime`.

## Architecture

The simulator implements the Alonso-Mora et al. (PNAS 2017) ride-sharing framework. Each iteration of the main loop (`src/main.cpp`) does:

```
buffer → generator (RTV graph + CTSP) → ilp_full → rebalance → simulator
```

1. **Buffer** (`src/buffer.cpp`) — releases newly available requests and active vehicles for the current time window
2. **Generator** (`src/generator.cpp`) — enumerates feasible trips per vehicle (RTV graph); most expensive step
3. **ILP Assignment** (`src/algorithms/ilp_full.cpp`) — solves fleet-wide assignment via Gurobi; LP relaxation variant in `ilp_full_linear.cpp`
4. **Rebalancer** (`src/rebalance.cpp`) — dispatches idle vehicles toward demand via a separate Gurobi MIP; creates `is_fake = true` placeholder trips
5. **Simulator** (`src/simulator.cpp`) — advances vehicles one time interval, processing boardings/alightings; parallelized via the thread pool

### Core Data Structures

- `Request` (`headers/request.hpp`) — passenger trip with origin/destination nodes, time windows (`latest_boarding`, `latest_alighting`), GPS coords, and multi-modal fields (`original_req_id`, `bus_trip_id`, `leg_type`, `bus_line_info`)
- `NodeStop` (`headers/request.hpp`) — a pickup or dropoff event: `(Request*, is_pickup, node)`; a sequence of these defines a vehicle route
- `Trip` (`headers/trip.hpp`) — a set of `Request*` assigned to one vehicle, with `cost`, `order_record` (ordered `NodeStop` list), `is_fake` (rebalancing placeholder), and `use_memory` (reuse cached route across iterations)
- `Vehicle` (`headers/vehicle.hpp`) — state machine with states `{Idle, Rebalancing, EnRoute, InUse, Boarding}`; tracks `node`, `offset` (partial progress to next node), `passengers`, `pending_requests`, `just_boarded`, `just_alighted`, and time-in-state accumulators
- `Network` (`headers/network.hpp`) — precomputed all-pairs time and distance matrices; `get_time(a,b)` is the primary lookup; `get_vehicle_time(v, node)` accounts for the vehicle's current in-transit `offset`

### CTSP Route Planner

`routeplanner::travel()` / `routeplanner::time_travel()` (`src/routeplanner.cpp`) find the optimal pickup/dropoff ordering for a vehicle + request set, subject to capacity, `latest_boarding`, `latest_alighting`, and dwell times. Controlled by `CTSP`:

| Strategy | Description |
|---|---|
| `FULL` | All permutations considered |
| `FIX_ONBOARD` | On-board passengers' stops fixed; new stops inserted around them |
| `FIX_PREFIX` | Entire current route prefix fixed; new stops appended/inserted at end (default) |
| `MEGA_TSP` | Large-scale TSP variant |

CTSP objective (`CTSP_OBJECTIVE`): `CTSP_VMT` (minimize vehicle miles, default), `CTSP_TOTALDROPOFFTIME`, or `CTSP_TOTALWAITING`.

### Assignment ILP

The ILP minimizes total trip cost with `MISS_COST = 10,000,000` penalizing unserved requests and `RMT_REWARD = 100` rewarding revenue miles. Constraints: each request served at most once, each vehicle assigned at most one trip. Feasible trips from the previous iteration are cached in `prev_trip_list` and reused where possible.

### Parallelism

`Threads` (`headers/threads.hpp`) wraps a C thread pool (`threadpool/thpool.c`). Two dispatch modes used internally:
- `auto_thread` — divides N jobs evenly across threads (batch CTSP enumeration)
- `mega_thread` — one task per job (finer-grained parallelism)

### Multi-Modal Extension

When `ALLOW_MULTI_MODAL=true`, `leg_requests` are loaded alongside standard requests. The CTSP constraint machinery enforces transit connection deadlines via `latest_alighting` naturally, making leg requests first-class citizens in assignment. Final statistics separately report direct, first-leg, last-leg, and both-leg service rates.

### Output Files (written to `RESULTS_DIRECTORY/`)

- `results.log` — per-iteration and final summary (service rate, avg waiting, avg riding, avg delay, mean occupancy, shared rate, assignment/ILP/rebalance wall times)
- `ilp.csv` — per-epoch ILP solve metrics: `Time, Obj, SolverTime, RelGap, NumAssigned, Status`
- `assignment.log` — per-epoch vehicle-to-request assignments
- `rebalance.log` — per-epoch rebalancing targets
- `unassigned_vehicles.log` — vehicles with no assignment each epoch

## Key Constants (compile-time, in `headers/settings.hpp`)

- `MISS_COST = 10000000.0` — penalty for unserved request in ILP objective
- `RMT_REWARD = 100.0` — reward for revenue miles traveled
- `PRUNING_RV_K = 30` — connect requests to nearest k vehicles only
- `PRUNING_RR_K = 30` — connect requests to nearest k other requests only
- `FIX_ASSIGNMENT_BEFORE = 300` — lock assignment when pickup is within 300s
- `MAX_REQ_PER_ITER = 500` — cap on requests processed per iteration

## Performance Notes

At scale (Chicago, ~2531 vehicles, capacity 4, `ILP_FULL`), each 30-second simulation interval takes ~40–50 seconds wall-clock time: ILP solve dominates at ~36–47s, rebalancing MIP ~1–4s. Large runs are designed for SLURM HPC cluster deployment (see `run_batch.sub`).
