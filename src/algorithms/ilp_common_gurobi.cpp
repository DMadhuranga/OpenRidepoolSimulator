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
        map<Vehicle*, vector<Trip>> const & trip_list, vector<Request*> const & requests, int time, double time_limit, map<Vehicle*, Trip> const & linear_assignment)
{
    // Simultaneously count variable, get cost vector, and build set for constraint 2.
    int K = 0; //requests.size();
    map<int, int> k_map;
    set<int> initial_sol_x;
    map<int, set<int>> reqs_to_legs;
    for (Request* request : requests)
    {
        if (request->original_req_id != -1) {
            int id = request->id;
            int original_req_id = request->original_req_id;
            reqs_to_legs[original_req_id].insert(id);
        }
        if (!request->assigned && request->original_req_id == -1){
            k_map[request->id] = K;
            initial_sol_x.insert(K);
            K ++;
        }
    }

    // miss_penalty = c * avg travel duration of current requests
    int epoch_n = 0;
    double epoch_duration = 0.0;
    for (Request* r : requests)
        if (r->original_req_id == -1) { epoch_n++; epoch_duration += r->ideal_traveltime; }
    double D = (epoch_n > 0) ? epoch_duration / epoch_n : 0.0;
    double miss_penalty = (D > 0.0) ? DEMAND_PENALTY_C * D : MISS_COST;
    int index = 0;
    vector<double> costs;
    map<int, set<int>> rids_to_trips;  // IRK + ITI

    set<int> initial_sol_e;  // IRK + ITI
    double initial_cost = 0;
    double prev_cost = 0;
    
    for (auto &id_trip_pair : trip_list)
    {
        vector<Trip> trips = id_trip_pair.second;
        Vehicle* v = id_trip_pair.first;
        set<Request*> prev (v->pending_requests.begin(), v->pending_requests.end());
        double prev_trip_cost = -1;
        if (linear_assignment.size())
        {
            prev.clear();
            if (linear_assignment.count(v))
            {
                Trip trip = linear_assignment.at(v);
                prev_trip_cost = trip.cost;
                if ((trip.requests.size() == 0 && v->passengers.size() == 0) && trip.cost != 0)
                    cout << "Non-zero empty trip. vid: " << v->id << " trip cost: " << prev_trip_cost << endl;
                prev_cost += trip.cost;
                for (Request* request : trip.requests)
                {
                    prev.insert(request);
                    int req_id = request->id;
                    if (request->original_req_id != -1)
                        req_id = request->original_req_id;
                    if (k_map.count(req_id))
                        initial_sol_x.erase(k_map[req_id]);
                }
            }
        }
        bool added_to_initial_sol = false;
        
        for (auto &trip : trips)
        {
            // cout << "Trip cost: " << trip.cost << "\t Req: ";
            vector<Request*> requests = trip.requests;
            bool contain_first_leg = false;
            for (Request* request : requests)
            {
                int id = request->id;
                // cout << id << "\t";
                rids_to_trips[id].insert(index);
                if (request->original_req_id != -1 && request->leg_type == 0 && !request->assigned)
                {
                    contain_first_leg = true;
                }
            }
            set<Request*> rs (trip.requests.begin(), trip.requests.end());
            if (!added_to_initial_sol && prev == rs)
            {
                initial_sol_e.insert(index);
                added_to_initial_sol = true;
                initial_cost += trip.cost;
                // if (linear_assignment.size() > 0)
                // {
                //     cout << "Cost mismatch for vid " << v->id << " trip cost: " << trip.cost << " prev_trip_cost: " << prev_trip_cost << endl;
                //     cout << "Current costs initial cost: " << initial_cost << " prev_cost: " << prev_cost << endl;
                //     for (Request* request : trip.requests)
                //         cout << request->id << "\t";
                //     cout << endl;
                //     if (prev_trip_cost != -1)
                //     {
                //         Trip assigned_trip = linear_assignment.at(v);
                //         for (Request* request : assigned_trip.requests)
                //             cout << request->id << "\t";
                //         cout << endl;
                //     }
                //     // throw runtime_error("Cost mismatch for vid " + to_string(v->id));
                // }
            }
            if (contain_first_leg)
                costs.push_back(trip.cost);
            else
                costs.push_back(trip.cost);
            index ++;
            // cout << endl;
        }
    }
    
    if (index == 0)
        return {};

    GRBVar* e = nullptr;
    GRBVar* x = nullptr;

    try {
    // Creating an environment
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", RESULTS_DIRECTORY + "/mip.log");
    env.start();

    // Create an empty model
    GRBModel model = GRBModel(env);

    e = new GRBVar[index];
    x = new GRBVar[K];
    for (auto i = 0; i < index; i++)
    {
        e[i] = model.addVar(0.0, 1.0, costs[i], GRB_BINARY, "e_"+itos(i));
        if (initial_sol_e.count(i))
        {
            e[i].set(GRB_DoubleAttr_Start, 1.0);
        } else {
            e[i].set(GRB_DoubleAttr_Start, 0.0);
        }
    }
    {
        ostringstream msg;
        msg << "Miss penalty: " << miss_penalty << " (D=" << D << ", n=" << epoch_n << ")";
        info(msg.str(), Yellow);
    }
    for (auto i = 0; i < K; i++)
    {
        x[i] = model.addVar(0.0, 1.0, miss_penalty, GRB_BINARY, "x_"+itos(i));
        if (initial_sol_x.count(i))
        {
            initial_cost += miss_penalty;
            prev_cost += miss_penalty;
            x[i].set(GRB_DoubleAttr_Start, 1.0);
        } else {
            x[i].set(GRB_DoubleAttr_Start, 0.0);
        }
    }

    // for (auto &id_trip_pair: linear_assignment) {
    //     Trip trip = id_trip_pair.second;
    //     prev_cost += trip.cost;
    // }

    cout << "Initial cost: " << initial_cost << " Previous cost: " << prev_cost << endl;

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
        
        model.addConstr(expr == 1, name);
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
        double start_sum = 0;
        for (int j : rids_to_trips[id]){
            expr += e[j];
            // if (j > index)
            //     cout << "j: " << j << " index: " << index << endl;
            // start_sum += e[j].get(GRB_DoubleAttr_Start);
        }
        // if (start_sum > 1){
        //     for (int j : rids_to_trips[id]){
        //         if (e[j].get(GRB_DoubleAttr_Start) > 0.5)
        //             cout << j << " = " << e[j].get(GRB_DoubleAttr_Start) << endl;
        //     }
        // }
        if (request->assigned) {
            model.addConstr(expr == 1, name);
            // rids_to_trips[id].insert(index);
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
                // if (leg_request->leg_type == 0)
                //     continue;
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

    // model.update();
    // if (linear_assignment.size() == 0)
    //     model.write("debug_lp.lp");
    // if (linear_assignment.size() > 0)
    //     model.write("debug.lp");
    
    // if (linear_assignment.size() > 0)
    // {
    //     for (int i = 0; i < index; i++) {
    //         if (e[i].get(GRB_DoubleAttr_Start) > 0.5)
    //             cout << i << " = " << e[i].get(GRB_DoubleAttr_Start) << endl;
    //     }
    // }
    
    // Warm start
    cout << "GUROBI_TIME_LIMIT: " << time_limit << endl;
    int old_sol_limit = model.get(GRB_IntParam_SolutionLimit);
    model.set(GRB_IntParam_Threads, 128);
    if (linear_assignment.size()) 
    {
        model.set(GRB_IntParam_MIPFocus, 2);
    } else {
        model.set(GRB_IntParam_MIPFocus, 1);
    }
    model.set(GRB_IntParam_Presolve, 1);
    // model.set(GRB_DoubleParam_Heuristics, 0.5);
    // model.set(GRB_IntParam_SolutionLimit, 1);
    model.set(GRB_DoubleParam_MIPGap, 2e-2);
    // model.set(GRB_DoubleParam_NoRelHeurTime, 10.0);
    // model.set(GRB_IntParam_Method, 1);
    model.set(GRB_DoubleParam_TimeLimit, time_limit);
    model.optimize();

    // Optimize
    // double remaining_time = time_limit - model.get(GRB_DoubleAttr_Runtime);
    // cout << "GUROBI_REMAINING_TIME_LIMIT: " << remaining_time << endl;
    // if (remaining_time > 0) {
        // model.set(GRB_IntParam_SolutionLimit, 1);
        // model.set(GRB_DoubleParam_NoRelHeurTime, 0.0);
    //     model.set(GRB_DoubleParam_MIPGap, 1e-4);
    //     model.set(GRB_IntParam_SolutionLimit, old_sol_limit);
    //     model.set(GRB_DoubleParam_TimeLimit, remaining_time);
    //     model.optimize();
    // }
    
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
    if (linear_assignment.size())
    {
        ofstream ilpfile(RESULTS_DIRECTORY + "/ilp.csv", std::ios_base::app);
        
        ilpfile << encode_time(time) << ",";
        ilpfile << model.get(GRB_DoubleAttr_ObjVal) << ",";
        ilpfile << model.get(GRB_DoubleAttr_Runtime) << ",";
        ilpfile << model.get(GRB_DoubleAttr_MIPGap) << ",";
        ilpfile << icount << ",";
        ilpfile << model.get(GRB_IntAttr_Status) << endl;
        // bool is_optimal = (M->getPrimalSolutionStatus() == SolutionStatus::NearOptimal ||
        //         M->getPrimalSolutionStatus() == SolutionStatus::Optimal);
        // ilpfile << (is_optimal ? "Optimal" : "Suboptimal") << endl;
    } else {
        ofstream ilpfile(RESULTS_DIRECTORY + "/linear_a_ilp.csv", std::ios_base::app);
        
        ilpfile << encode_time(time) << ",";
        ilpfile << model.get(GRB_DoubleAttr_ObjVal) << ",";
        ilpfile << model.get(GRB_DoubleAttr_Runtime) << ",";
        ilpfile << model.get(GRB_DoubleAttr_MIPGap) << ",";
        ilpfile << icount << ",";
        ilpfile << model.get(GRB_IntAttr_Status) << endl;
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

    if (linear_assignment.size()>0 && prev_cost != initial_cost) {
        cout << "Initial cost does not match previous cost" << endl;
    }

    // Free dynamically allocated memory
    delete[] e;
    delete[] x;
    
    return assigned_trips;
    
    } catch (GRBException ex) {
        cout << "GRBException occurred" << endl;
        cout << "Error code = " << ex.getErrorCode() << endl;
        cout << ex.getMessage() << endl;
        // Free dynamically allocated memory before returning
        if (e != nullptr) delete[] e;
        if (x != nullptr) delete[] x;
        // Return empty assignment map on error
        return map<Vehicle*, Trip>();
    }
}

}
