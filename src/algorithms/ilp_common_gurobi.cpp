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

#include <chrono>
#include "formatting.hpp"
#include "settings.hpp"

#include <fstream>
#include "gurobi_c++.h"            // For gurobi functions.
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

// using namespace mosek::fusion;  // For MOSEK functions.
// using namespace monty;          // For MOSEK functions.
using namespace std;

string itos(int i) {stringstream s; s << i; return s.str(); }
namespace ilp_common_gurobi
{

double last_prep_seconds        = 0.0;
double last_model_build_seconds = 0.0;
double last_solve_seconds       = 0.0;
double last_extract_seconds     = 0.0;

namespace {
    std::chrono::steady_clock::time_point _tp;
    inline void _tic() { _tp = std::chrono::steady_clock::now(); }
    inline double _toc() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - _tp).count() / 1e6;
    }
}

/* Function to solve main assignment problem. */
map<Vehicle*,Trip> ilp_assignment_gurobi(
        map<Vehicle*, vector<Trip>> const & trip_list, vector<Request*> const & requests, int time, double time_limit, map<Vehicle*, Trip> const & linear_assignment, bool primary_stats)
{
    last_prep_seconds = last_model_build_seconds = 0.0;
    last_solve_seconds = last_extract_seconds = 0.0;
    _tic();
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
    // Was map<int,set<int>>; only ever looked up by id, so dense slots give the
    // same constraints without a tree descent per (trip, request) pair.
    unordered_map<int,int> rid_idx;
    rid_idx.reserve(requests.size() * 2);
    for (Request* r : requests)
    {
        int nxt = (int) rid_idx.size();
        rid_idx.emplace(r->id, nxt);
    }
    vector<vector<int>> rids_to_trips (rid_idx.size());
    static vector<int> const no_trips;   // for ids not among `requests`
    auto trips_of = [&rid_idx, &rids_to_trips](int id) -> vector<int> const & {
        auto it = rid_idx.find(id);
        return it == rid_idx.end() ? no_trips : rids_to_trips[it->second];
    };
    // Replaces three O(requests.size()) scans nested in the constraint loops.
    unordered_map<int,Request*> req_by_id;
    req_by_id.reserve(requests.size() * 2);
    for (Request* r : requests)
        req_by_id.emplace(r->id, r);

    // One push_back per trip, ~1.6 M of them, so size it once.
    {
        size_t total_trips = 0;
        for (auto & p : trip_list) total_trips += p.second.size();
        costs.reserve(total_trips);
    }

    set<int> initial_sol_e;  // IRK + ITI
    double initial_cost = 0;
    double prev_cost = 0;
    
    for (auto &id_trip_pair : trip_list)
    {
        vector<Trip> const & trips = id_trip_pair.second;
        Vehicle* v = id_trip_pair.first;
        set<Request*> prev (v->pending_requests.begin(), v->pending_requests.end());
        double prev_trip_cost = -1;
        if (linear_assignment.size())
        {
            prev.clear();
            if (linear_assignment.count(v))
            {
                Trip const & trip = linear_assignment.at(v);
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
            vector<Request*> const & requests = trip.requests;
            for (Request* request : requests)
            {
                int id = request->id;
                // cout << id << "\t";
                auto rit = rid_idx.find(id);
                if (rit == rid_idx.end())
                    continue;   // such an entry was never read back
                // `index` is non-decreasing, so a repeat can only be back(): set semantics.
                auto & slot = rids_to_trips[rit->second];
                if (slot.empty() || slot.back() != index)
                    slot.push_back(index);
            }
            // Build the set only when the match is still possible; a larger `prev` can't match.
            if (!added_to_initial_sol && prev.size() <= trip.requests.size()
                && prev == set<Request*>(trip.requests.begin(), trip.requests.end()))
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
            // Both branches of the former contain_first_leg test were identical.
            costs.push_back(trip.cost);
            index ++;
            // cout << endl;
        }
    }
    
    if (index == 0)
    {
        last_prep_seconds = _toc();
        return {};
    }
    last_prep_seconds = _toc();
    _tic();

    GRBVar* e = nullptr;
    GRBVar* x = nullptr;

    try {
    // Creating an environment
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", RESULTS_DIRECTORY + "/mip.log");
    env.start();

    // Create an empty model
    GRBModel model = GRBModel(env);

    // Batched: was 1.6 M addVar calls, each naming the column via a stringstream.
    // Column order is load-bearing (the solution is read back positionally) and
    // addVars appends in array order, so it is preserved exactly.
    {
        vector<char>   vtypes (index, GRB_BINARY);
        vector<double> vlb (index, 0.0), vub (index, 1.0);
        e = model.addVars(vlb.data(), vub.data(), costs.data(), vtypes.data(), nullptr, index);
        vector<double> estart (index, 0.0);
        for (int i : initial_sol_e)
            estart[i] = 1.0;
        model.set(GRB_DoubleAttr_Start, e, estart.data(), index);
    }
    x = new GRBVar[K];
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

    // Every constraint is a unit sum, and `expr += e[j]` allocated a GRBLinExpr
    // temporary per term (~7.0 M/epoch); addTerms appends them in bulk.
    vector<double> ones;
    {
        size_t maxlen = 0;
        for (auto & p : trip_list) maxlen = max(maxlen, p.second.size());
        for (auto & v : rids_to_trips) maxlen = max(maxlen, v.size());
        ones.assign(maxlen, 1.0);
    }
    vector<GRBVar> gather;   // reused scratch for the non-contiguous sums
    gather.reserve(ones.size());

    // Constraint One.
    size_t count = 0;
    for (auto &id_trip_pair : trip_list)
    {
        int vid = id_trip_pair.first->id;
        size_t const n_trips = id_trip_pair.second.size();  // was a full vector copy
        string name = "c1-" + to_string(vid);

        // One vehicle's trips are contiguous columns, so no gather is needed.
        GRBLinExpr expr = 0;
        expr.addTerms(ones.data(), e + count, (int) n_trips);
        
        model.addConstr(expr == 1, name);
        count += n_trips;
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
        gather.clear();
        for (int j : trips_of(id)){
            gather.push_back(e[j]);
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
            if (ones.size() < gather.size()) ones.assign(gather.size(), 1.0);
            expr.addTerms(ones.data(), gather.data(), (int) gather.size());
            model.addConstr(expr == 1, name);
            // rids_to_trips[id].insert(index);
        } else {
            set<int> considered_bus_trips;
            for (int leg_id : reqs_to_legs[id]) 
            {
                Request* leg_request = req_by_id.at(leg_id);
                // if (leg_request->leg_type == 0)
                //     continue;
                int bus_trip_id = leg_request->bus_trip_id;
                if (considered_bus_trips.count(bus_trip_id)==0)
                {
                    for (int j : trips_of(leg_id))
                        gather.push_back(e[j]);
                }
                considered_bus_trips.insert(bus_trip_id);
            }
            if (ones.size() < gather.size()) ones.assign(gather.size(), 1.0);
            expr.addTerms(ones.data(), gather.data(), (int) gather.size());
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
            Request* first_leg_request = req_by_id.at(leg_id);
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
            for (int j : trips_of(leg_id))
                expr += e[j];
            for (int j : trips_of(last_leg_request_id))
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
    // Was hardcoded to 1; presolve measured 57% of solve time.  Default 1 is unchanged.
    model.set(GRB_IntParam_Presolve, GRB_PRESOLVE);
    // model.set(GRB_DoubleParam_Heuristics, 0.5);
    // model.set(GRB_IntParam_SolutionLimit, 1);
    model.set(GRB_DoubleParam_MIPGap, 2e-2);
    // model.set(GRB_DoubleParam_NoRelHeurTime, 10.0);
    // model.set(GRB_IntParam_Method, 1);
    model.set(GRB_DoubleParam_TimeLimit, time_limit);
    last_model_build_seconds = _toc();
    _tic();
    model.optimize();
    last_solve_seconds = _toc();
    _tic();

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
    // Write statistics.  Destination is explicit; see the header.
    if (primary_stats)
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
    
    last_extract_seconds = _toc();
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
