# OpenRidepoolSimulator — Code and Data Report

## Overview

**OpenRidepoolSimulator** is a C++ discrete-time simulation of an on-demand shared ride (ridepooling) service. It was written by Vindula Jayawardana and Matthew Zalesak (MIT License, 2018–2020), with multimodal extensions added subsequently. The simulator models a fleet of vehicles dispatched over a road network, optimally assigning multiple passenger requests to shared vehicles using Integer Linear Programming (ILP) via the Gurobi solver.

The `info` file contains a SLURM cluster node listing, indicating the simulator was designed to run on a high-performance computing cluster — consistent with the compute-intensive nature of solving ILPs at scale.

---

## Repository Structure

```
OpenRidepoolSimulator/
├── headers/
│   ├── request.hpp          # Request and NodeStop structs
│   ├── vehicle.hpp          # Vehicle class
│   ├── trip.hpp             # Trip struct
│   ├── network.hpp          # Road network class
│   ├── simulator.hpp        # Vehicle state advancement
│   ├── buffer.hpp           # Input staging (active vehicles/requests)
│   ├── generator.hpp        # RTV graph / feasible trip enumeration
│   ├── routeplanner.hpp     # CTSP solver
│   ├── rebalance.hpp        # Fleet rebalancing
│   ├── threads.hpp          # Thread pool wrapper
│   ├── csvreader.hpp        # Data loading
│   ├── formatting.hpp       # Time encoding and logging utilities
│   ├── settings.hpp         # Global configuration parameters
│   └── algorithms/
│       ├── ilp_full.hpp           # ILP assignment (with memory)
│       ├── ilp_full_linear.hpp    # LP relaxation assignment
│       └── ilp_common_gurobi.hpp  # Gurobi solver interface
├── src/
│   ├── vehicle.cpp
│   ├── csvreader.cpp
│   ├── formatting.cpp
│   └── threads.cpp
├── threadpool/
│   ├── thpool.c             # C thread pool implementation
│   └── thpool.h
├── data/chicago/
│   ├── map/                 # nodes.csv, edges.csv, times.csv
│   └── vehicles/vehicles.csv
├── outputs/chicago/         # Simulation output logs
└── mip.cpp                  # Standalone Gurobi test file
```

---

## Core Data Structures

### `Request` (`headers/request.hpp`)
Represents a single passenger trip request:
- **Origin/destination**: as graph node IDs and GPS coordinates
- **Timing constraints**: `entry_time`, `latest_boarding`, `latest_alighting` (hard deadlines for pickup and dropoff)
- **Multimodal fields**: `bus_trip_id`, `leg_type`, `bus_line_info` — used when the trip is a first/last-mile leg connecting to a bus line
- Derived field: `ideal_traveltime` (shortest-path travel time, used to compute detour allowance and to back-calculate `latest_boarding` from `latest_alighting`)

### `NodeStop` (`headers/request.hpp`)
A pickup or dropoff event: a `(Request*, is_pickup, node)` tuple. A sequence of `NodeStop`s defines a vehicle's route.

### `Vehicle` (`headers/vehicle.hpp`, `src/vehicle.cpp`)
Models one vehicle in the fleet:
- **State machine**: `{Idle, Rebalancing, EnRoute, InUse, Boarding}` — state transitions are timestamped to accumulate time-in-state statistics
- **Position**: current `node` and `offset` (partial progress toward the next node)
- **Passenger lists**: `passengers` (currently aboard), `pending_requests` (assigned but not yet picked up), `just_boarded`, `just_alighted`
- **Distance tracking**: total distance traveled and rebalancing distance tracked separately

### `Trip` (`headers/trip.hpp`)
A candidate assignment: a set of `Request*`s, a `cost`, a route (`order_record` of `NodeStop`s), and flags:
- `is_fake`: marks a rebalancing placeholder trip (no real passengers)
- `use_memory`: allows reusing a previously computed route across iterations

### `Network` (`headers/network.hpp`)
The road graph:
- Preloaded **time matrix** and **distance matrix** (all-pairs, from `times.csv` and `edges.csv`)
- Adjacency list supporting Dijkstra's algorithm for path queries
- Helper methods account for a vehicle's current in-transit `offset` when computing arrival times

---

## System Architecture

The simulation runs as a discrete-time loop. Each iteration covers one time interval (configured by `INTERVAL`) and runs the following pipeline:

```
buffer → generator → ilp_full → rebalance → simulator
```

### 1. `buffer` — Input Staging
- `get_active_vehicles`: returns vehicles whose `start_time` has elapsed
- `get_new_requests`: releases requests that have arrived by the current time, from both standard and leg-request pools

### 2. `generator` — Feasible Trip Enumeration
Builds the Request-Trip-Vehicle (RTV) graph: for each vehicle, enumerates feasible combinations of requests it could serve. This is the most computationally expensive step.
- Uses `routeplanner::travel()` to solve the Constrained TSP (CTSP) for each candidate request set, verifying all timing constraints (max waiting, max detour) are satisfied
- Feasible trips from the previous iteration are cached as `prev_trip_list` and reused where possible
- Pruning heuristics (`PRUNING_RV_K`, `PRUNING_RR_K`) limit which vehicle–request pairs are considered to reduce enumeration cost

### 3. `routeplanner` — CTSP Solver (`headers/routeplanner.hpp`)
Given a vehicle and a set of requests, finds the optimal pickup/dropoff ordering subject to:
- Vehicle capacity constraint (`CARSIZE`)
- Per-request `latest_boarding` and `latest_alighting` deadlines
- Optional dwell times at pickup (`DWELL_PICKUP`) and dropoff (`DWELL_ALIGHT`)

Multiple CTSP strategies are supported (configured via `CTSP`):

| Strategy | Description |
|---|---|
| `FULL` | All pickup/dropoff permutations considered |
| `FIX_ONBOARD` | On-board passengers' stops are fixed; new stops inserted around them |
| `FIX_PREFIX` | Entire current route prefix fixed; new stops appended/inserted at the end |
| `MEGA_TSP` | Large-scale TSP variant |

CTSP objectives (`CTSP_OBJECTIVE`):
- `CTSP_VMT` — minimize vehicle miles traveled
- `CTSP_TOTALDROPOFFTIME` — minimize total passenger drop-off time
- `CTSP_TOTALWAITING` — minimize total passenger waiting time

### 4. `ilp_full` / `ilp_full_linear` — Global Assignment (`headers/algorithms/`)
Takes the RTV graph and solves the fleet-wide assignment problem:
- **ILP mode** (`ilp_full`): calls Gurobi via `ilp_common_gurobi::ilp_assignment_gurobi()` to solve the integer program exactly (subject to a time limit)
- **Linear relaxation mode** (`ilp_full_linear`): solves the LP relaxation — used as a warm-start for the ILP or as a fast standalone assignment
- The ILP minimizes total trip cost subject to: each request served at most once, each vehicle assigned at most one trip
- `MISS_COST = 10,000,000` penalizes unserved requests heavily; `RMT_REWARD = 100` rewards revenue miles traveled

### 5. `rebalance` — Fleet Rebalancing (`headers/rebalance.hpp`)
After assignment, idle vehicles are dispatched toward high-demand areas via a separate MIP (also Gurobi-based). A "fake" trip (`is_fake = true`) is created to represent a rebalancing move with no passengers.

### 6. `simulator` — State Advancement (`headers/simulator.hpp`)
Advances each vehicle forward by one time interval: updates positions, processes boardings and alightings, and updates vehicle state and statistics. Parallelized using the `Threads` pool.

---

## Parallelism

The `Threads` class (`headers/threads.hpp`, `src/threads.cpp`) wraps a C thread pool (`threadpool/thpool.c`). Two dispatch strategies:
- **`auto_thread`**: divides N jobs evenly across the thread pool (used for batch CTSP enumeration)
- **`mega_thread`**: creates one task per job (for finer-grained parallelism)

---

## Configuration (`headers/settings.hpp`)

All simulation parameters are global `extern` variables initialized via command-line through `initialize()`:

| Parameter | Example Value | Description |
|---|---|---|
| `MAX_WAITING` | 1200 s | Maximum wait before pickup |
| `MAX_DETOUR` | 1.2× | Maximum ride time relative to direct trip |
| `CARSIZE` | 4 | Vehicle passenger capacity |
| `GUROBI_TIME_LIMIT` | 180 s | Per-iteration ILP time budget |
| `PRUNING_RV_K` | 30 | Nearest-k vehicles considered per request |
| `PRUNING_RR_K` | 30 | Nearest-k requests considered per request |
| `FIX_ASSIGNMENT_BEFORE` | 300 s | Lock assignment when pickup is within this many seconds |
| `LINEAR_ASSIGNMENT` | bool | Use LP relaxation instead of ILP |
| `ALLOW_MULTI_MODAL` | bool | Enable first/last-mile leg mode |
| `DISABLE_REASSIGNMENT` | bool | Prevent re-assigning vehicles with committed routes |
| `DISABLE_DIRECT_TRIPS` | bool | Disallow single-passenger (non-shared) trips |
| `PRE_SOLVE_ILP` | bool | Warm-start ILP with LP solution |
| `INTERVAL` | int | Simulation time step in seconds |
| `INITIAL_TIME` / `FINAL_TIME` | HHMMSS int | Simulation window |

---

## Multimodal (First/Last-Mile) Extensions

The multimodal feature was added to the original simulator to support integrated ridepooling + public transit trips. When `ALLOW_MULTI_MODAL` is enabled, the simulator loads an additional `leg_requests` file alongside the standard requests.

### Concept
A multimodal trip consists of:
1. **First leg** (optional): ridepooling vehicle takes the passenger from their origin to a bus/transit stop
2. **Transit leg**: passenger rides the bus (not simulated — treated as a fixed schedule)
3. **Last leg** (optional): ridepooling vehicle picks the passenger up from a transit stop and delivers them to their destination

Some trips have only a first leg (the bus stop is near the destination), only a last leg (the bus stop is near the origin), or both.

### Leg Request Data Format

The leg requests are precomputed by a separate pipeline (in `scripts-for-simulator`) and stored as CSV files.

**Header**: `leg_re_id, origin, origin_lon, origin_lat, dest, dest_lon, dest_lat, earliest_pickup_time, latest_dropoff_time, original_trip_request_id, bus_trip_id, leg_type, info`

| Field | Description |
|---|---|
| `leg_re_id` | Unique ID for this leg request (e.g., 900000+) |
| `origin` | Map node ID of pickup location |
| `origin_lon`, `origin_lat` | GPS coordinates of pickup |
| `dest` | Map node ID of dropoff location |
| `dest_lon`, `dest_lat` | GPS coordinates of dropoff |
| `earliest_pickup_time` | Earliest time the passenger is ready for pickup (HH:MM:SS) |
| `latest_dropoff_time` | Hard deadline for dropoff — must arrive by this time to make the bus (maps to `latest_alighting` in simulator) |
| `original_trip_request_id` | ID linking this leg to its parent trip in `requests.csv` |
| `bus_trip_id` | Identifies the specific bus run this leg connects to |
| `leg_type` | `0` = first leg (to transit), `1` = last leg (from transit) |
| `info` | Human-readable transit route string (bus lines, run IDs, intermediate stop coords and transfer times) |

**Example rows** (Boston dataset):
```
900000,35673,-71.1464,42.3643,51985,-71.1445,42.3617,06:00:02,06:09:00,68,0,0,Bus lines: 86_0 runs: 1
900008,6857,-71.0339,42.3887,8590,-71.0685,42.3741,06:00:02,06:23:00,62,6,0,Bus lines: Orange_0-Green-E_0 runs: 6-11 ...
900010,2504,-71.0573,42.3568,56221,-71.0829,42.3458,06:25:00,06:45:42,62,8,1,Bus lines: 111_1-93_1 runs: 15-1 ...
```

### How the Simulator Uses Leg Requests

In `csvreader.cpp`, leg requests are loaded with `first_last_legs = true`:
- `latest_alighting` is set directly from `latest_dropoff_time`
- `latest_boarding` is back-calculated as `latest_alighting - ideal_traveltime` (the simulator works backward from the hard bus deadline)
- `bus_line_info` stores the raw `info` string (not parsed further by the simulator)
- `original_req_id`, `bus_trip_id`, and `leg_type` are stored on the `Request` struct

The CTSP constraint checking in `routeplanner` naturally enforces the transit connection deadline via `latest_alighting`, making multimodal trips first-class citizens in the assignment.

---

## Input Data

### Road Network
- `data/chicago/map/times.csv` — all-pairs travel time matrix between map nodes
- `data/chicago/map/edges.csv` — road edges with travel cost
- `data/chicago/map/nodes.csv` — node coordinates

### Vehicles (`vehicles.csv`)
Format: `driver_id, starting_node, latitude, longitude, start_time, capacity`

Example:
```
1,4088,41.8186,-87.6478,00:00:00,4
2,27966,41.9303,-87.7074,00:00:00,4
```
All vehicles start at time 0; capacity defaults to `CARSIZE` unless `CARSIZE < 0` (then per-vehicle capacity from file is used).

### Trip Requests (`requests.csv`)
Format: `request_id, origin_node, origin_lon, origin_lat, dest_node, dest_lon, dest_lat, requested_time`

Example:
```
4,6266,-71.0246,42.3989,50429,-70.9847,42.4507,06:00:00
```
`latest_boarding` is set to `entry_time + MAX_WAITING`; `latest_alighting` to `entry_time + MAX_WAITING + MAX_DETOUR * ideal_traveltime`.

---

## Outputs

Per time-interval logs (`results.log`) report:

| Metric | Description |
|---|---|
| `Service Rate` | Fraction of requests served so far |
| `Avg Waiting` | Average time from request to pickup |
| `Avg Riding` | Average in-vehicle time |
| `Avg Delay` | Average excess ride time vs. direct trip |
| `Mean Passen` | Average vehicle occupancy |
| `Shared Rate` | Fraction of trips shared with another passenger |
| `Pending Requests` | Current backlog of unassigned requests |
| `Assignment Time` | Wall-clock time for the full assignment step |
| `ILP Assignment Time` | Wall-clock time spent in Gurobi |
| `Rebalance Time` | Wall-clock time for rebalancing MIP |

Additional per-vehicle logs track actions, assignments, rebalancing moves, and unserved requests.

### Observed Performance (Chicago, 2531 vehicles, capacity 4, ILP_FULL)
Each 30-second simulation interval takes approximately 40–50 seconds of wall-clock time:
- ILP solve: ~36–47 s (dominant cost)
- Rebalancing MIP: ~1–4 s

This confirms the need for HPC cluster deployment for large-scale runs.

---

## `mip.cpp` — Standalone Gurobi Test

A standalone binary MIP example from Gurobi's own documentation, used to verify the Gurobi installation and license. It is not part of the main simulation.

---

## Summary

OpenRidepoolSimulator is a research-grade, modular ridesharing simulation framework implementing the RTV-graph + ILP approach to fleet-level ridepooling assignment (following the methodology of Alonso-Mora et al., 2017).

Key design decisions:
- **Gurobi ILP** as the assignment engine, with LP relaxation as a fast alternative or warm-start
- **CTSP-based route planning** with configurable constraint-fixing strategies for tractability
- **Multimodal first/last-mile extensions** enabling integrated transit + ridepooling simulation, with hard bus connection deadlines enforced through the existing CTSP constraint machinery
- **Parallel trip enumeration** via a C thread pool
- **HPC cluster deployment** (SLURM) due to long solve times at scale
- Data available for **Chicago** (primary) and **Boston** (multimodal experiments)
