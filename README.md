# OpenRidepoolSimulator

OpenRidepoolSimulator is a C++ framework for studying algorithms for ride-pooling assignment problems. Its modular design makes it easy to swap out individual components — assignment algorithms, rebalancing policies, CTSP heuristics, and RTV graph parameters — while keeping the rest of the simulation intact.

The base implementation follows the trip-vehicle assignment framework from:

> Alonso-Mora et al., "On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment", *PNAS* 2017.

This repository extends that baseline with **multi-modal transit integration**, implementing the methodology from:

> Edirimanna, Hu & Samaranayake, "Integrating On-demand Ride-sharing with Mass Transit at-Scale", *arXiv* 2404.07691 (2024).

In this extension, on-demand shuttle vehicles serve first-mile and/or last-mile legs that connect passengers to and from fixed-route mass transit. Transit legs are pre-planned and encoded as `leg_requests`; the assignment ILP treats them as first-class requests with hard connection deadlines, so the existing CTSP feasibility machinery enforces transit timing constraints without modification.

The framework supports:

* Batching intervals and simulation time windows
* Swappable map, vehicle, and request inputs (five U.S. cities included)
* Custom assignment algorithms and rebalancing policies
* Pluggable CTSP subproblem heuristics
* Multi-modal (first/last-mile + transit) trip generation and assignment

The software is tested on Linux with the G++ compiler.

## Dependencies

- [Gurobi](https://www.gurobi.com/) optimizer (set the `GUROBI_HOME` environment variable to your Gurobi installation directory)
- [Boost](https://www.boost.org/) C++ libraries

## Input Data

Datasets for the five included cities ship as archives in `data/`, one per city:

| archive | nodes | edges | requests | transit legs | `times.npy` once generated |
|---|---|---|---|---|---|
| `data/atlanta_inputs.zip` | 54,652 | 129,731 | 15,332 | 62,531 | 6.0 GB |
| `data/boston_inputs.zip` | 62,832 | 152,834 | 30,383 | 62,942 | 7.9 GB |
| `data/chicago_inputs.zip` | 44,095 | 120,275 | 28,518 | 392,291 | 3.9 GB |
| `data/houston_inputs.zip` | 70,775 | 176,946 | 18,369 | 21,665 | 10.0 GB |
| `data/la_inputs.zip` | 76,531 | 212,432 | 47,006 | 100,017 | 11.7 GB |

### 1. Extract

Unpack an archive into `data/`, which puts the city directory where `DATAROOT`
will point:

```bash
unzip data/chicago_inputs.zip -d data/      # -> data/chicago/...
```

Each archive holds everything the simulator reads except the travel-time
matrix:

```
<city>/map/edges.csv              directed road graph, "origin,dest,travel_time" (1-based node ids)
<city>/requests/requests.csv      direct (non-transit) requests
<city>/requests/leg_requests.csv  first/last-mile transit legs, read when ALLOW_MULTI_MODAL is true
<city>/vehicles/vehicles.csv      vehicle fleet; VEHICLE_LIMIT caps how many are loaded
<city>/map/nodes.csv              node coordinates; reference only, never read by the simulator
```

### 2. Generate `times.npy`

The simulator also needs `<city>/map/times.npy`, a dense uint16 all-pairs
travel-time matrix. It is derived data — the all-pairs shortest paths over
`map/edges.csv` — and far larger than its source, 3.9 GB from a 1.7 MB edge
list for Chicago, so it is generated locally instead of distributed:

```bash
python3 data/gen_times_npy.py data/chicago              # ~7 min single-threaded
python3 data/gen_times_npy.py data/chicago --jobs 8     # ~1 min
```

Requires `numpy` and `scipy`. The script writes through a memory-mapped output,
so the matrix never has to fit in RAM, and it reports any unreachable node
pairs or distances that overflow uint16 rather than discarding them silently.

To check a matrix against a known-good copy — every sampled row, cell by cell:

```bash
python3 data/gen_times_npy.py data/chicago --verify-only \
  --verify /path/to/known-good/times.npy --samples 25
```

Two things to know before running: the simulator aborts while constructing
`Network` if `times.npy` is absent, and loading the matrix costs about twice
the file size in RAM, since `network.cpp` expands each uint16 entry to `int`
(7.8 GB resident for Chicago, 23 GB for LA).

## Compiling

```bash
export GUROBI_HOME=/path/to/gurobi  # e.g. /opt/gurobi1100/linux64
make
```

This produces the `./prog` binary. To remove build artifacts:

```bash
make clean
```

To run the simulator, in the console enter

```bash
./prog <num_threads> DATAROOT <path> RESULTS_DIRECTORY <path> [key value ...]
```

Example:

```bash
./prog 4 DATAROOT data/chicago RESULTS_DIRECTORY results \
  INTERVAL 30 MAX_WAITING 1200 MAX_DETOUR 1.2 CARSIZE 4 \
  INITIAL_TIME 60000 FINAL_TIME 190000 \
  GRB_TIME_LIMIT 20 RTV_TIMELIMIT 10000 \
  ALLOW_MULTI_MODAL false DISABLE_REASSIGNMENT false
```

where the first argument is the number of threads the simulator may use in parallel. All remaining arguments are key-value pairs. Full list of options (defaults in `src/settings.cpp`):

**Data and I/O**
* `DATAROOT` - (default `"data"`) root directory for input data
* `RESULTS_DIRECTORY` - (default `"results"`) directory to write output files to
* `REQUEST_DATA_FILE` - (default `"requests.csv"`) request file within `DATAROOT/requests/`
* `VEHICLE_DATA_FILE` - (default `"vehicles.csv"`) vehicle file within `DATAROOT/vehicles/`
* `TIMEFILE_NPY` - (default `"times.npy"`) binary uint16 travel-time matrix within `DATAROOT/map/`

**Fleet and requests**
* `CARSIZE` - (default `4`) maximum passengers per vehicle; if negative, per-vehicle capacity from `vehicles.csv` is used
* `VEHICLE_LIMIT` - (default `1000`) cap on vehicles loaded from the vehicle file
* `MAX_WAITING` - maximum waiting time in seconds for served passengers
* `MAX_DETOUR` - maximum detour as a factor of direct travel time
* `INITIAL_TIME` - (default `0`) simulation start time in HHMMSS format
* `FINAL_TIME` - (default `240000`) simulation end time in HHMMSS format
* `INTERVAL` - (default `60`) seconds between assignment epochs

**Solver**
* `GRB_TIME_LIMIT` - (default `90`) Gurobi solver time limit in seconds per epoch
* `GRB_LICENSE_FILE` - path to Gurobi license file
* `RTV_TIMELIMIT` - (default `0`, unlimited) milliseconds per vehicle for RTV graph generation
* `DEMAND_PENALTY_C` - (default `1`) scaling constant `c` for the unserved-request penalty; penalty = `c × avg_travel_duration` of current requests
* `LINEAR_ASSIGNMENT` - use LP relaxation instead of full ILP (default `false`)
* `PRE_SOLVE_ILP` - warm-start ILP with LP solution (default `false`)
* `CTSP` - route planner variant: `FULL`, `FIX_ONBOARD`, `FIX_PREFIX` (default), `MEGA_TSP`

**Assignment behaviour**
* `DISABLE_REASSIGNMENT` - (default `false`) prevent re-assigning vehicles that have a committed route
* `DISABLE_DIRECT_TRIPS` - (default `false`) disallow direct trips that does not use transit

**Multi-modal**
* `ALLOW_MULTI_MODAL` - (default `false`) enable multi-modal trips (first/last-mile + transit)
* `ONLY_ALLOW_SINGLE_LEG` - (default `false`) when multi-modal is enabled, skip transit options that require both a first leg and a last leg, allowing single-leg connections only

## Citation

If you use this software, please cite both the original framework and the multi-modal extension:

```
Alonso-Mora, J., Samaranayake, S., Wallar, A., Frazzoli, E., & Rus, D. (2017).
On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment.
PNAS, 114(3), 462–467.

Edirimanna, D., Hu, H., & Samaranayake, S. (2024).
Integrating On-demand Ride-sharing with Mass Transit at-Scale.
arXiv:2404.07691.
```

## Credits

The base simulator was produced by Matthew Zalesak and Vindula Jayawardana. The multi-modal transit extension was developed by Danushka Edirimanna.
