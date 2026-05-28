# OpenRidepoolSimulator

OpenRidepoolSimulator is a C++ framework designed to aid the study of algorithms for ridepool assignment problems.  Its modular design is intended to make it easy to modify only the components one is interested in exploring and changing.  The base implementation closely follows the design presented in "On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment" (Alonso-Mora et al, PNAS 2017).

The design allows flexibility to control:

* Batching intervals
* Map, vehicle, and request inputs
* Easily create your own assignment algorithms
* Easily create you own rebalancing policy
* Insert you own CTSP subproblem heuristic
* Easily change parameters for RTV graph generation

The software is tested on Linux with the G++ compiler.

## Dependencies

- [Gurobi](https://www.gurobi.com/) optimizer (set the `GUROBI_HOME` environment variable to your Gurobi installation directory)
- [Boost](https://www.boost.org/) C++ libraries

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
* `DISABLE_DIRECT_TRIPS` - (default `false`) disallow single-passenger (non-shared) trips

**Multi-modal**
* `ALLOW_MULTI_MODAL` - (default `false`) enable multi-modal trips (first/last-mile + transit)
* `ONLY_ALLOW_SINGLE_LEG` - (default `false`) when multi-modal is enabled, skip transit options that require both a first leg and a last leg, allowing single-leg connections only

This software was produced by Matthew Zalesak and Vindula Jayawardana.
