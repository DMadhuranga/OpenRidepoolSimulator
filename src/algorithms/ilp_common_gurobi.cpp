/*
 * The MIT License
 *
 * Copyright 2020 Matthew Zalesak.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "algorithms/ilp_common_gurobi.hpp"
#include "formatting.hpp"
#include "settings.hpp"

#include <fstream>
#include "gurobi_c++.h"            // For gurobi functions.
#include <set>
#include <sstream>

// using namespace mosek::fusion;  // For MOSEK functions.
// using namespace monty;          // For MOSEK functions.
using namespace std;

string itos(int i) {stringstream s; s << i; return s.str(); }
namespace ilp_common_gurobi
{

/* Function to solve main assignment problem. */
map<Vehicle*,Trip> ilp_assignment_gurobi(
        map<Vehicle*, vector<Trip>> const & trip_list, vector<Request*> const & requests, int time)
{
    // Simultaneously count variable, get cost vector, and build set for constraint 2.
    int K = 0; //requests.size();
    map<int, set<int>> reqs_to_legs;
    for (Request* request : requests)
    {
        if (request->original_req_id != -1) {
            int id = request->id;
            int original_req_id = request->original_req_id;
            reqs_to_legs[original_req_id].insert(id);
        }
        if (!request->assigned && request->original_req_id == -1)
            K ++;
    }
    int index = 0;
    vector<double> costs;
    map<int, set<int>> rids_to_trips;  // IRK + ITI
    
    for (auto &id_trip_pair : trip_list)
    {
        vector<Trip> trips = id_trip_pair.second;
        
        for (auto &trip : trips)
        {
            // cout << "Trip cost: " << trip.cost << "\t Req: ";
            costs.push_back(trip.cost);
            vector<Request*> requests = trip.requests;
            for (Request* request : requests)
            {
                int id = request->id;
                // cout << id << "\t";
                rids_to_trips[id].insert(index);
            }
            index ++;
            // cout << endl;
        }
    }
    
    if (index == 0)
        return {};

    // Creating an environment
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", RESULTS_DIRECTORY + "/mip.log");
    env.start();

    // Create an empty model
    GRBModel model = GRBModel(env);

    GRBVar* e = new GRBVar[index];
    GRBVar* x = new GRBVar[K];
    for (auto i = 0; i < index; i++)
    {
        e[i] = model.addVar(0.0, 1.0, costs[i], GRB_BINARY, "e_"+itos(i));
    }
    for (auto i = 0; i < K; i++)
    {
        x[i] = model.addVar(0.0, 1.0, MISS_COST, GRB_BINARY, "x_"+itos(i));
    }

    // Constraint One.
    int count = 0;
    for (auto &id_trip_pair : trip_list)
    {
        int vid = id_trip_pair.first->id;
        vector<Trip> trips = id_trip_pair.second;
        string name = "c1-" + to_string(vid);

        GRBLinExpr expr = 0;
        for (int j = count; j < count + trips.size(); j++)
            expr += e[j];
        
        model.addConstr(expr <= 1, name);
        count += trips.size();
    }
    
    // Constraint Two (request should be served via an option or penalty term set to 1).
    int k = 0;
    for (Request* request : requests)
    {
        if (!request->assigned && request->original_req_id != -1)
            continue; // constraint is per original request
        int id = request->id;
        string name = "c2-" + to_string(id);
        
        GRBLinExpr expr = 0;
        for (int j : rids_to_trips[id])
            expr += e[j];
        if (request->assigned) {
            model.addConstr(expr == 1, name);
            rids_to_trips[id].insert(index);
        } else {
            set<int> considered_bus_trips;
            for (int leg_id : reqs_to_legs[id]) 
            {
                Request* leg_request;
                for (Request* req : requests)
                {
                    if (req->id == leg_id) {
                        leg_request = req;
                        break;
                    }
                }
                int bus_trip_id = leg_request->bus_trip_id;
                if (considered_bus_trips.count(bus_trip_id)==0)
                {
                    for (int j : rids_to_trips[leg_id])
                        expr += e[j];
                }
                considered_bus_trips.insert(bus_trip_id);
            }
            model.addConstr(expr + x[k] == 1, name);
            k ++;
        }
    }

    // Constraint Three (Both legs of an multi modal option should be served).
    for (Request* request : requests)
    {
        // only unassigned requests are allowed for mode changes
        if (request->assigned)
            continue;
        
        // consider only the original request
        if (request->original_req_id != -1)
            continue;

        int id = request->id;
        for (int leg_id : reqs_to_legs[id]) {
            Request* first_leg_request;
            for (Request* req : requests)
            {
                if (req->id == leg_id) {
                    first_leg_request = req;
                    break;
                }
            }
            // only consider first leg requests
            if (first_leg_request->leg_type == 1)
                continue;
            int bus_trip_id = first_leg_request->bus_trip_id;

            int last_leg_request_id = -1;
            for (Request* req : requests)
            {
                if (req->bus_trip_id == bus_trip_id && req->leg_type == 1) {
                    last_leg_request_id = req->id;
                    break;
                }
            }

            // only consider multi-modal trips with both legs
            if (last_leg_request_id == -1)
                continue;

            GRBLinExpr expr = 0;
            for (int j : rids_to_trips[leg_id])
                expr += e[j];
            for (int j : rids_to_trips[last_leg_request_id])
                expr -= e[j];
            string name = "c3-" + to_string(bus_trip_id);
            model.addConstr(expr == 0, name);
        }
    }

    {
        int i = 0;
        for (auto r : requests)
            if (r->assigned)
                i++;
        cout << "Number of assigned requests: " << i << "/" << requests.size() << endl;
    }
    
    // Solve.
    model.set(GRB_DoubleParam_TimeLimit, GRB_TIME_LIMIT);
    model.optimize();
    
    vector<int> assignments;
    int icount = 0;
    
    for (auto i = 0; i < index; i++)
    {
        double d = e[i].get(GRB_DoubleAttr_X);
        assignments.push_back(d > 0.5);
        icount += (d > 0.5);
    }
    cout << "Made " << icount << " assignments." << endl;
    // Write statistics.
    {
        ofstream ilpfile(RESULTS_DIRECTORY + "/ilp.csv", std::ios_base::app);
        
        ilpfile << encode_time(time) << "\t";
        ilpfile << model.get(GRB_DoubleAttr_ObjVal) << "\t";
        ilpfile << model.get(GRB_DoubleAttr_Runtime) << "\t";
        ilpfile << model.get(GRB_DoubleAttr_MIPGap) << "\t";
        ilpfile << icount << "\t";
        ilpfile << model.get(GRB_IntAttr_Status) << endl;
        // bool is_optimal = (M->getPrimalSolutionStatus() == SolutionStatus::NearOptimal ||
        //         M->getPrimalSolutionStatus() == SolutionStatus::Optimal);
        // ilpfile << (is_optimal ? "Optimal" : "Suboptimal") << endl;
    }
    
    map<Vehicle*, Trip> assigned_trips;
    count = 0;
    for (auto & x : trip_list)
    {
        Vehicle* v = x.first;
        vector<Trip> const* trips = &x.second;
        for (auto r = 0; r < trips->size(); r++)
            if (assignments[r + count] > 0.5)
            {
                assigned_trips[v] = (*trips)[r];
                break;
            }
        
        count += trips->size();
    }
    
    return assigned_trips;
}

}
