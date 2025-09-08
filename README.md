# OpenRidepoolSimulator

OpenRidepoolSimulator is a C++ framework designed to aid the study of algorithms for ridepool assignment problems.  Its modular design is intended to make it easy to modify only the components one is interested in exploring and changing.  The base implementation closely follows the design presented in "On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment" (Alonso-Mora et al, PNAS 2017).

The design allows flexibility to control:

* Batching intervals
* Map, vehicle, and request inputs
* Easily create your own assignment algorithms
* Easily create you own rebalancing policy
* Insert you own CTSP subproblem heuristic
* Easily change parameters for RTV graph generation

The software is tested on linux with G++ compiler.  Optimization is written with Mosek Optimizer version 8.1.0.56 and may not be compatible with later or current versions of the Mosek.

To compile the program, simply enter the directory and run the "make" command.

To run the simulator, in the console enter

```

ALLOW_MULTI_MODAL=true 
DISABLE_REASSIGNMENT="false"
DISABLE_DIRECT_TRIPS="false"
INTERVAL=30
MAX_ADD_COST=4
RTV_TIMELIMIT=10000
GRB_TIME_LIMIT=20
INITIAL_TIME=60000
FINAL_TIME=190000
./prog $no_veh DATAROOT $DATAROOT RESULTS_DIRECTORY $RESULTS_DIRECTORY RTV_TIMELIMIT $RTV_TIMELIMIT MAX_ADD_COST $MAX_ADD_COST VEHICLE_LIMIT $VEHICLE_LIMIT CARSIZE $CARSIZE GRB_TIME_LIMIT $GRB_TIME_LIMIT INTERVAL $INTERVAL MAX_WAITING 1200 MAX_DETOUR 1.2 DWELL_PICKUP 0 DWELL_ALIGHT 0 ALLOW_MULTI_MODAL $ALLOW_MULTI_MODAL DISABLE_DIRECT_TRIPS $DISABLE_DIRECT_TRIPS DISABLE_REASSIGNMENT "$DISABLE_REASSIGNMENT" INITIAL_TIME $INITIAL_TIME FINAL_TIME $FINAL_TIME

```

where x is the number of threads the simulator may use in parallel.  Addiitonally, you may include the following arguments, all given as keywords followed by values.  For example, to run the program with 500 vehicles you would use

```
./prog x VEHICLE_LIMIT 500
```

They keywords include (more listed in file settings.cpp):

* ALLOW_MULTI_MODAL - Allow multi modal trips
* DISABLE_REASSIGNMENT - Disable reassignment of requests to another vehicle
* DISABLE_DIRECT_TRIPS - Disable direct trips (only multi-modal trip is allowed)
* GRB_LICENSE_FILE - Gurobi license file path
* GRB_TIME_LIMIT - Time limit given for ILP solver
* DATAROOT - (default "./data") location to look for simulation inputs
* RESULTS_DIRECTORY - (default "results") location to write results to, ignores if folder not found
* VEHICLE_LIMIT - (default no limit) maximum number of vehicles to load from vehicle file.
* MAX_WAITING - maximum waiting time for served passengers
* MAX_DETOUR - maximum detour for served passengers as a factor of direct travel time
* REQUEST_DATA_FILE - (default requests.csv) Input request file within DATAROOT/requests/
* CARSIZE - (default 4) maximum number of passengers per vehicle
* INITIAL_TIME - (default 0) starting time of simulation given as HHMMSS.
* FINAL_TIME - (default 24000) ending time of simulation given as HHMMSS.
* INTERVAL - (default 60) time that passes between subsequent assignment epochs
* RTV_TIMELIMIT - (default 0) number of miliseconds the RTV graph generator can spend on each vehicle

This software was produced by Matthew Zalesak and Vindula Jayawardana.
