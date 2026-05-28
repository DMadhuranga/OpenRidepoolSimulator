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
#include "generator.hpp"
#include "routeplanner.hpp"
#include "settings.hpp"

#include "gurobi_c++.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex> // <-- Guilty party.  Secretly includes "chrono"
#include <random>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace std;

namespace ilp_full
{

mutex mtx;

// Rolling performance statistics maintained across iterations.
// Updated at the end of each assignment() call and consumed by prune_rvgraph()
// in the following iteration to estimate realistic serving capacity.
static deque<int>    stat_n_served;       // new requests served per iteration
static deque<double> stat_p_served;       // service rate of new requests
static deque<double> stat_trips_per_veh;  // assigned (non-fake) trips / num vehicles
static const int     STAT_HISTORY = 10;

struct rtv_thread_data
{
    int time;
    map<Request*, set<Request*>>* rr_edges;
    map<Vehicle*, vector<Request*>>* vr_edges;
    map<Vehicle*, vector<Trip>>* trip_list;
    Network const* network;
    vector<Vehicle*> const* vehicles;
    std::map<Vehicle*, std::vector<Trip>> const* prev_trip_list;
};


//Function to create RTV graph
void make_rtvgraph(void* rtv_data)
{
    bool rtvverbose = false;
    
    struct thread_data* t = (struct thread_data*) rtv_data;
    int start = t->start;
    int end = t->end;
    
    struct rtv_thread_data* data = (struct rtv_thread_data*) t->data;
    auto time = data->time;
    auto rr_edges = data->rr_edges;
    auto vr_edges = data->vr_edges;
    auto trip_list = data->trip_list;
    auto network = data->network;
    auto vehicles = data->vehicles;
    auto prev_trip_list = data->prev_trip_list;
    
    for (auto i = start; i < end; i ++)
    {
        // Boiler plate for timing.
        auto start_time = chrono::steady_clock::now();
        bool timeout = false; // Note:  Use RTV_TIMELIMIT in settings.hpp to control.
        
        // Select the vehicle, make our clique list by iteration k.
        Vehicle* v = (*vehicles)[i];
        stringstream outputs;
        outputs << "Starting RTV for vid " << v->id << endl;
        vector<vector<Trip>> round;
        set<Request*> previous_assigned_passengers (v->pending_requests.begin(), v->pending_requests.end());
        
        // Generate initial trip with no assignment.
        {
            Trip baseline {};
            vector<Request*> rs;
            auto result = routeplanner::time_travel(*v, rs, STANDARD, *network, time, start_time);
            baseline.cost = result.first;
            baseline.order_record = result.second;
            round.push_back(vector<Trip>({baseline}));
            round[0][0].requests.clear();
        }
        
        // Validate and update previous trips since vehicle may have moved
        map<set<Request*>, Trip> validated_trip_cache;
        set<int> previous_feasible_request_ids;
        if (prev_trip_list->count(v)) {
            int valid_count = 0;
            int invalid_count = 0;
            int reused_path_count = 0;
            
            // Check if vehicle state has changed since last iteration
            bool vehicle_state_unchanged = v->just_boarded.empty() && v->just_alighted.empty();
            
            for (auto & prev_trip : prev_trip_list->at(v)) {
                // Recompute the trip to check if it's still feasible
                vector<Request*> trip_requests = prev_trip.requests;
                if (trip_requests.size() == 1) {
                    previous_feasible_request_ids.insert(trip_requests[0]->id);
                }
                
                pair<int,vector<NodeStop>> recomputed_path;
                bool quick_check_passed = false;
                
                if (vehicle_state_unchanged) {
                    quick_check_passed = routeplanner::check_order_record_feasibility(
                        *v, prev_trip.order_record, *network, time);
                }
                
                if (quick_check_passed) {
                    // Quick check passed and vehicle state unchanged - reuse cached path
                    recomputed_path.first = prev_trip.cost;
                    recomputed_path.second = prev_trip.order_record;
                    reused_path_count++;
                } else {
                    // Either quick check failed or vehicle state changed - do full recomputation
                    recomputed_path = routeplanner::time_travel(
                        *v, trip_requests, STANDARD, *network, time, start_time);
                }
                
                if (recomputed_path.first >= 0) {
                    // Trip is still valid - update with new cost and order_record
                    Trip validated_trip {};
                    validated_trip.cost = recomputed_path.first;
                    validated_trip.order_record = recomputed_path.second;
                    validated_trip.requests = trip_requests;
                    validated_trip.is_fake = prev_trip.is_fake;
                    validated_trip.use_memory = prev_trip.use_memory;
                    
                    set<Request*> trip_reqs(trip_requests.begin(), trip_requests.end());
                    validated_trip_cache[trip_reqs] = validated_trip;
                    valid_count++;
                } else {
                    invalid_count++;
                }
            }
            outputs << "Validated " << valid_count << " trips, " << invalid_count 
                    << " trips became infeasible, " << reused_path_count << " trips reused paths for vid " << v->id << endl;
        }

        mtx.lock();
        set<Request*> initial_pairing ((*vr_edges)[v].begin(), (*vr_edges)[v].end());
        mtx.unlock();
        
        round.push_back(vector<Trip>());
        initial_pairing.insert(v->pending_requests.begin(), v->pending_requests.end());
        if (initial_pairing.size() > (*vr_edges)[v].size())
        {
            outputs << "Added "; 
            for (auto r : initial_pairing)
            {
                if (find((*vr_edges)[v].begin(), (*vr_edges)[v].end(), r) == (*vr_edges)[v].end()) 
                {
                    outputs << r->id << ",";
                }
            }
            outputs << " reqs to vid " << v->id << endl;
        }
        int total_pax = (int)previous_assigned_passengers.size() + (int)v->passengers.size();
        bool skip_fresh_generation = (total_pax >= SKIP_FRESH_PAX_THRESHOLD);

        for (auto r : initial_pairing)
        {
            vector<Request*> requests {r};
            set<Request*> request_set {r};

            // Check if this trip was already validated
            if (validated_trip_cache.count(request_set)) {
                round[1].push_back(validated_trip_cache[request_set]);
                outputs << r->id << "(cached),";
            } else {
                pair<int,vector<NodeStop>> path = routeplanner::time_travel(*v, requests, STANDARD, *network, time, start_time);
                if (path.first >= 0)
                {
                    Trip trip {};
                    trip.cost = path.first;
                    trip.order_record = path.second;
                    trip.requests = {r};
                    round[1].push_back(trip);
                    outputs << r->id << ",";
                }
            }
        }
        outputs << endl;

        int existing_trip_size = previous_assigned_passengers.size();
        outputs << "Number of assigned passengers: " << existing_trip_size << endl;
        outputs << "Number of passengers on board: " << v->passengers.size() << endl;
        if (skip_fresh_generation)
            outputs << "Skip fresh generation (total_pax=" << total_pax << ")" << endl;
        // In all subsequent rounds, take pairs from the previous round and build if they add one new element.
        int counter = 0;
        while (round.size() <= existing_trip_size + 2 || (round[round.size() - 1].size() && !timeout))
        {
            int k = round.size();
            if (k > 3*v->capacity)
                break;
            round.push_back(vector<Trip>());

            if (k == existing_trip_size)
            {
                Trip previoustrip = generator::previoustrip(v, *network, time);
                if (previoustrip.cost < 0)
                {
                    cout << "Negative cost for vid " << v->id << endl;
                    cout << "Current time " << time << endl;
                    cout << "v.node " << v->node << endl;
                    int current_node = v->node;
                    int current_time = time;
                    for (NodeStop n : v->order_record)
                    {
                        cout << "order_record " << n.r->id << "," << n.is_pickup << "," << n.node << endl;
                        cout << "latest_alighting " << n.r->latest_alighting << endl;
                        cout << "latest_boarding " << n.r->latest_boarding << endl;
                        current_time += network->get_time(current_node, n.node);
                        cout << "Current time " << current_time << endl;
                        if (n.is_pickup && current_time < n.r->entry_time)
                            current_time = n.r->entry_time;
                        current_node = n.node;
                    }
                    throw runtime_error("Previous assignment no longer feasible. Vid: "+to_string(v->id));
                }
                round[k].push_back(previoustrip);
            }

            // Add trips from validated_trip_cache (previous round) with k requests
            for (auto & trip : validated_trip_cache) {
                if (trip.first.size() == k && trip.first != previous_assigned_passengers) {
                    round[k].push_back(trip.second);
                }
            }

            if (DISABLE_REASSIGNMENT && k <= existing_trip_size)
                continue;

            for (auto first = 0; first < round[k - 1].size(); first++)
            {
                // always allow to build on top of previous assignment.
                if (timeout && (k != existing_trip_size+1 || first > 0))
                    break;
                // Get new request set.
                set<Request*> left (round[k-1][first].requests.begin(), round[k-1][first].requests.end());
                int prev_round = k - 1;
                int starting_second = first + 1;
                if (k == existing_trip_size+1 && (DISABLE_REASSIGNMENT || timeout)) {
                    prev_round = 1;
                    starting_second = 0;
                }
                
                for (auto second = starting_second; second < round[prev_round].size(); second++)
                {
                    // Check the time.
                    auto end_time = chrono::steady_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds> (end_time - start_time).count();
                    if ((k != existing_trip_size+1 || first > 0) && RTV_TIMELIMIT && duration > RTV_TIMELIMIT && !timeout)
                    {
                        outputs << "timeout" << endl;
                        timeout = true;
                        if (k != existing_trip_size+1) break;
                    }

                    // Get new request set.
                    set<Request*> right (round[prev_round][second].requests.begin(), round[prev_round][second].requests.end());
                    // if (k == existing_trip_size + 1)
                    // {
                    //     for (auto r : right)
                    //         outputs << r->id << ",";
                    //     outputs << "right" << endl;
                    // }
                    set<Request*> requests = left;
                    requests.insert(right.begin(), right.end());
                    
                    counter ++;
                    
                    // Reject if there are no new requests not in previous_feasible_request_ids
                    bool has_new_request = false;
                    for (auto r : requests) {
                        if (!previous_feasible_request_ids.count(r->id)) {
                            has_new_request = true;
                            break;
                        }
                    }
                    if (!has_new_request) {
                        continue;
                    }
                    // Reject if there are too many new requests.
                    int const MAX_NEW = 8;
                    {
                        int max_new = MAX_NEW;
                        for (auto r : requests)
                            if (!previous_assigned_passengers.count(r))
                                max_new -= 2;
                        if (max_new < 0)
                            continue;
                    }
                    
                    // Reject if this is not a simple +1.
                    if (requests.size() != k)
                        continue;

                    // For heavily loaded vehicles, skip trips that contain no existing
                    // assigned passengers — only build on top of prior assignments.
                    if (skip_fresh_generation) {
                        bool has_existing = false;
                        for (auto r : requests)
                            if (previous_assigned_passengers.count(r)) { has_existing = true; break; }
                        if (!has_existing) continue;
                    }

                    // Reject if this is not a unique trip.
                    bool unique = true;
                    for (auto & t : round[k])
                        if (set<Request*>(t.requests.begin(), t.requests.end()) == requests)
                        {
                            unique = false;
                            break;
                        }
                    if (!unique)
                        continue;

                    // if (k == existing_trip_size + 1)
                    // {
                    //     for (auto r : requests)
                    //         outputs << r->id << ",";
                    //     outputs << "considered" << endl;
                    // }
                    
                    // Add a placeholder to show we've considered this option.
                    vector<Request*> request_vector (requests.begin(), requests.end());
                    round[k].push_back({-1, false, false, {}, request_vector});
                    
                    // Reject if the RR graph does not connect the requests.
                    bool rr_connected = true;
                    for (auto r : left)
                        if (!right.count(r))
                            for (auto rr : right)
                                if (!(*rr_edges)[r].count(rr) && !(*rr_edges)[rr].count(r))
                                {
                                    rr_connected = false;
                                    break;
                                }
                    if (!rr_connected)
                        continue;
                    for (auto r : right)
                        if (!left.count(r))
                            for (auto rr : left)
                                if (!(*rr_edges)[r].count(rr) && !(*rr_edges)[rr].count(r))
                                {
                                    rr_connected = false;
                                    break;
                                }
                    if (!rr_connected)
                        continue;

                    // if (k == existing_trip_size + 1)
                    // {
                    //     for (auto r : requests)
                    //         outputs << r->id << ",";
                    //     outputs << "rr_connected" << endl;
                    //     outputs << "first " << first << endl;
                    // }

                    // Reject if all subsets (-1) are not present.
                    bool subset_test = true;

                    for (auto r : requests)
                    {
                        if (DISABLE_REASSIGNMENT && previous_assigned_passengers.count(r))
                            continue;
                        // if (k == existing_trip_size + 1 && timeout && previous_assigned_passengers.count(r))
                        if (k == existing_trip_size + 1 && first==0 && timeout && previous_assigned_passengers.count(r))
                            continue;
                        set<Request*> subset = requests;
                        subset.erase(r);
                        bool matched = false;
                        for (auto & t : round[k - 1])
                            if (set<Request*>(t.requests.begin(), t.requests.end()) == subset)
                            {
                                matched = true;
                                break;
                            }
                        if (!matched)
                        {
                            subset_test = false;
                            break;
                        }
                    }
                    if (!subset_test)
                        continue;

                    // if (k == existing_trip_size + 1)
                    // {
                    //     for (auto r : requests)
                    //         outputs << r->id << ",";
                    //     outputs << "preokay" << endl;
                    // }
                    // Reject if there is no feasible routing for this request set.
                    bool preokay = true;
                    // if (k != existing_trip_size + 1 && first > 0)
                    // {
                    //     auto end_time = chrono::steady_clock::now();
                    //     auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
                    //     preokay = (duration <= RTV_TIMELIMIT);
                    // }
                    pair<int,vector<NodeStop>> path;
                    if (validated_trip_cache.count(requests)) {
                        // Use validated trip from previous computation
                        Trip cached_trip = validated_trip_cache[requests];
                        path.first = cached_trip.cost;
                        path.second = cached_trip.order_record;
                    } else {
                        if (k == existing_trip_size + 1){
                            path = routeplanner::travel(
                                    *v, request_vector, STANDARD, *network, time);
                        } else {
                            path = routeplanner::time_travel(
                                    *v, request_vector, STANDARD, *network, time, start_time);
                        }
                    }
                    if (path.first < 0)
                        continue;

                    Request* new_request;
                    for (auto & r: right){
                        if (left.count(r) == 0) 
                        {
                            new_request = r;
                            break;
                        }
                    }
                    string reqs = "";
                    for (auto r: requests) {
                        reqs = reqs + to_string(r->id) + "\t";
                    }
                    string paths = "";
                    for (auto & node: path.second) {
                        paths = paths + to_string(node.r->id) + "\t";
                    }
                    // outputs << "Got cost: " << path.first << "\t Vid: " << v->id << "\t Requests: " << reqs << "Path: " << paths <<endl;
                    // outputs << first << "\t" << round[k-1].size() << endl;
                    // outputs << new_request->ideal_traveltime << endl;
                    // int seperate_cost = round[k-1][first].cost + new_request->ideal_traveltime;
                    // if (path.first > MAX_ADD_COST*(seperate_cost))
                    //     continue;

                    // for (auto r : requests)
                    //     outputs << r->id << ",";
                    // outputs << endl;
                    
                    // Accepted!  Save this new trip!
                    Trip trip {};
                    trip.cost = path.first;
                    trip.order_record = path.second;
                    trip.requests = vector<Request*>(requests.begin(), requests.end());
                    round[k][round[k].size() - 1] = trip;
                }
            }
            
            for (auto i = 0; i < round[k].size(); i++) // Filter failed placeholders.
                if (round[k][i].cost < 0)
                {
                    round[k][i] = round[k][round[k].size() - 1];
                    round[k].pop_back();
                    i--;
                }

            auto duration = chrono::duration_cast<chrono::milliseconds> (chrono::steady_clock::now() - start_time).count();
            double duration_seconds = duration / 1000.0; // Convert milliseconds to seconds
            outputs << "Current trip generation time (s): " << duration_seconds << endl;
            outputs << "Trip size: " << k << ", number of trips: " << round[k].size() << endl;
        }
        
        // Convert into appropriate format.  This include adding in the pending requests.
        vector<Trip> potential_trip_list;
        for (auto & list : round)
            potential_trip_list.insert(potential_trip_list.end(), list.begin(), list.end());
        
        for (auto & t : potential_trip_list)
            if (t.cost == -1) {
                string reqs = "";
                for (auto r: t.requests) {
                    reqs = reqs + to_string(r->id) + "\t";
                }
                throw runtime_error("Negative cost not cleaned up for vid " + to_string(v->id) + " requests " + reqs);
            }

        // Include possibility of previous assignment.
        if (v->order_record.size())
        {
            Trip previoustrip = generator::previoustrip(v, *network, time);
            potential_trip_list.push_back(previoustrip);

            int obj = previoustrip.cost;
            if (obj == -1)
                throw runtime_error("Previous assignment no longer feasible.");
        }
        
        mtx.lock();
        (*trip_list)[v] = potential_trip_list;
        mtx.unlock();

        // Check the time.
        auto duration = chrono::duration_cast<chrono::milliseconds> (chrono::steady_clock::now() - start_time).count();
        double duration_seconds = duration / 1000.0; // Convert milliseconds to seconds
        outputs << "RTV construction time (s): " << duration_seconds << endl;
        outputs << "Ended RTV for vid " << v->id << endl;
        mtx.lock();
        {
            ofstream debuglogfile (RESULTS_DIRECTORY + "/debug.log", std::ios_base::app);
            debuglogfile << outputs.rdbuf();
        }
        mtx.unlock();

    }
}


struct rv_thread_data
{
    int time;
    map<Request*, vector<Vehicle*>>* rv_edges;
    Network const* network;
    vector<Request*> const* requests;
    vector<Vehicle*> const* vehicles;
};


void make_rvgraph(void* rv_data)
{
    struct thread_data* t = (struct thread_data*) rv_data;
    int start = t->start;
    int end = t->end;

    struct rv_thread_data* data = (struct rv_thread_data*) t->data;
    auto time = data->time;
    auto rv_edges = data->rv_edges;
    auto network = data->network;
    auto requests = data->requests;
    auto vehicles = data->vehicles;

    for (int i = start; i < end; i++)
    {
        Request* r = (*requests)[i];
        vector<Request*> req_list { r };
        int origin = r->origin;
        vector<Vehicle*> compatible_vehicles;

        if (!DISABLE_DIRECT_TRIPS || r->original_req_id != -1)
        {
            // Pre-filter: only consider vehicles that can reach the origin in time.
            multimap<int, Vehicle*> nearest_vs;
            for (Vehicle* v : *vehicles)
            {
                int min_wait = network->get_vehicle_time(*v, origin);
                if (time + min_wait > r->latest_boarding) continue;
                nearest_vs.insert(make_pair(min_wait, v));
            }

            for (auto& x : nearest_vs)
            {
                Vehicle* v = x.second;
                pair<int, vector<NodeStop>> raw_path =
                    routeplanner::travel(*v, req_list, STANDARD, *network, time);
                if (raw_path.first >= 0)
                    compatible_vehicles.push_back(v);
            }
        }

        mtx.lock();
        (*rv_edges)[r] = compatible_vehicles;
        mtx.unlock();
    }
}


// Prune the full RV graph using a balanced, priority-aware selection.
// Requests are processed in urgency order (most constrained first).
// For each request, up to PRUNING_RV_K vehicles are selected probabilistically,
// favouring vehicles with fewer existing assignments to balance fleet load.
map<Request*, vector<Vehicle*>> prune_rvgraph(
    map<Request*, vector<Vehicle*>> const& rv_edges,
    vector<Request*> const& requests,
    vector<Vehicle*> const& vehicles,
    set<int> const& fixed_requests,
    int time)
{
    if (PRUNING_RV_K <= 0)
        return rv_edges;

    // Compute type-aware slack and slack/travel ratio for each request.
    // last-leg: slack = latest_alighting - ideal_traveltime - max(entry_time, current_time)
    // direct / first-leg: slack = latest_boarding - current_time
    map<Request*, double> ratio_map;
    map<Request*, int>    rv_size_map;
    for (auto r : requests)
    {
        double slack;
        if (r->leg_type == 1)
            slack = r->latest_alighting - r->ideal_traveltime
                    - (double)max(r->entry_time, time);
        else
            slack = r->latest_boarding - (double)time;

        double ttime = max(1, r->ideal_traveltime);
        ratio_map[r]   = slack / ttime;
        rv_size_map[r] = rv_edges.count(r) ? (int)rv_edges.at(r).size() : 0;
    }

    // Rank requests ascending by ratio and by rv_size (rank 0 = most urgent).
    vector<Request*> sorted_by_ratio = requests;
    sort(sorted_by_ratio.begin(), sorted_by_ratio.end(),
         [&](Request* a, Request* b) { return ratio_map[a] < ratio_map[b]; });

    vector<Request*> sorted_by_size = requests;
    sort(sorted_by_size.begin(), sorted_by_size.end(),
         [&](Request* a, Request* b) { return rv_size_map[a] < rv_size_map[b]; });

    map<Request*, int> rank_ratio, rank_size;
    for (int i = 0; i < (int)requests.size(); i++)
    {
        rank_ratio[sorted_by_ratio[i]] = i;
        rank_size[sorted_by_size[i]]   = i;
    }

    // Identify new requests entering this iteration (not previously assigned).
    int num_new = 0;
    for (auto r : requests)
        if (!r->assigned && r->entry_time >= time - INTERVAL)
            num_new++;

    // Estimate how many requests we can realistically serve this iteration using
    // rolling averages from the last STAT_HISTORY iterations:
    //   n_ave        — avg new requests served
    //   p_ave        — avg new-request service rate
    //   tpv_ave      — avg assigned trips per vehicle
    // estimated = max(tpv_ave * num_vehicles, n_ave, p_ave * num_new_this_iter)
    // Falls back to serving all requests when no history exists yet.
    int estimated_capacity = (int)requests.size();
    if (!stat_n_served.empty())
    {
        double n_ave = 0, p_ave = 0, tpv_ave = 0;
        for (int x    : stat_n_served)      n_ave   += x;
        for (double x : stat_p_served)      p_ave   += x;
        for (double x : stat_trips_per_veh) tpv_ave += x;
        n_ave   /= stat_n_served.size();
        p_ave   /= stat_p_served.size();
        tpv_ave /= stat_trips_per_veh.size();

        double est = max({tpv_ave * (double)vehicles.size(),
                          n_ave,
                          p_ave * (double)max(1, num_new)});
        estimated_capacity = (int)ceil(est);
    }

    // Build priority_set: the estimated_capacity easiest-to-serve unassigned requests.
    //
    // "Easiness" = high slack-ratio (plenty of schedule flexibility) AND many
    // compatible vehicles (many options). We rank by descending sum of rank_ratio
    // and rank_size — both ranks are ascending (0 = hardest/fewest options), so a
    // higher sum means easier.
    //
    // We walk the sorted list and add one request per unique passenger until we
    // reach estimated_capacity covered passengers. This prevents multiple
    // multi-modal alternatives for the same passenger from consuming capacity slots.
    // The remaining unassigned requests form the "hard" group (tier-2) and only
    // get vehicles after the easy group and previously assigned requests are served.
    set<Request*> priority_set;
    {
        vector<Request*> unassigned;
        for (auto r : requests)
            if (!r->assigned && !fixed_requests.count(r->id))
                unassigned.push_back(r);

        sort(unassigned.begin(), unassigned.end(),
             [&](Request* a, Request* b) {
                 return (rank_ratio[a] + rank_size[a]) > (rank_ratio[b] + rank_size[b]);
             });

        set<int> covered_passengers;
        for (auto r : unassigned)
        {
            if ((int)covered_passengers.size() >= estimated_capacity) break;
            int key = (r->original_req_id == -1) ? r->id : r->original_req_id;
            if (covered_passengers.count(key)) continue;
            covered_passengers.insert(key);
            priority_set.insert(r);
        }
    }

    // Three-tier processing order (lower tier = processed first):
    //   Tier 0 — easy-to-serve requests (priority_set, sized to estimated capacity)
    //   Tier 1 — previously assigned requests (vehicle pre-guaranteed)
    //   Tier 2 — hard-to-serve requests (remaining unassigned, get leftover vehicles)
    // Within each tier, sort hardest-first (lowest urgency score) so the most
    // constrained requests within the group get first pick of available vehicles.
    const double w1 = 1.0, w2 = 1.0;
    auto tier = [&](Request* r) -> int {
        if (priority_set.count(r)) return 0;
        if (r->assigned)           return 1;
        return 2;
    };
    vector<Request*> processing_order = requests;
    sort(processing_order.begin(), processing_order.end(),
         [&](Request* a, Request* b) {
             int ta = tier(a), tb = tier(b);
             if (ta != tb) return ta < tb;
             // Within tier: hardest first (ascending rank score = most constrained)
             return (w1 * rank_ratio[a] + w2 * rank_size[a]) <
                    (w1 * rank_ratio[b] + w2 * rank_size[b]);
         });

    // Track how many requests each vehicle has been selected for so far.
    map<Vehicle*, int> load;
    for (auto v : vehicles) load[v] = 0;

    mt19937 rng(42);
    map<Request*, vector<Vehicle*>> pruned;

    // Find the vehicle currently serving a request.
    auto find_assigned_vehicle = [&](Request* r) -> Vehicle*
    {
        for (auto v : vehicles)
            for (auto rp : v->pending_requests)
                if (rp->id == r->id) return v;
        return nullptr;
    };

    // Pre-populate: already-assigned requests always retain their current vehicle.
    // This guarantees continuity of service regardless of subsequent pruning.
    // Seed the load map so these vehicles are deprioritised for other requests.
    for (auto r : requests)
    {
        if (r->assigned)
        {
            Vehicle* fv = find_assigned_vehicle(r);
            if (fv)
            {
                pruned[r].push_back(fv);
                load[fv]++;
            }
        }
    }

    for (auto r : processing_order)
    {
        // Fixed requests (close to pickup): pre-populated vehicle is the only one needed.
        if (fixed_requests.count(r->id))
            continue;

        const vector<Vehicle*>& candidates =
            rv_edges.count(r) ? rv_edges.at(r) : vector<Vehicle*>{};

        // Initialise selection from any vehicle already in pruned[r] (assigned vehicle).
        vector<Vehicle*> selected    = pruned[r];
        set<Vehicle*>    selected_set(selected.begin(), selected.end());

        while ((int)selected.size() < PRUNING_RV_K)
        {
            // Build weight vector over candidates not yet selected for this request.
            vector<pair<Vehicle*, double>> weighted;
            double total_weight = 0.0;
            for (auto v : candidates)
            {
                if (selected_set.count(v)) continue;
                double w = 1.0 / (1.0 + load[v]);
                weighted.push_back({v, w});
                total_weight += w;
            }
            if (weighted.empty()) break;

            // Sample one vehicle proportional to 1/(1 + load).
            uniform_real_distribution<double> dist(0.0, total_weight);
            double roll = dist(rng);
            double cumulative = 0.0;
            Vehicle* chosen = weighted.back().first;
            for (auto& p : weighted)
            {
                cumulative += p.second;
                if (roll < cumulative) { chosen = p.first; break; }
            }

            selected.push_back(chosen);
            selected_set.insert(chosen);
            load[chosen]++;
        }

        pruned[r] = selected;
    }

    return pruned;
}


double detour_factor(const Request* a, const Request* b, const Network* network)
{
    double best = INFINITY;
    int o1 = a->origin, o2 = b->origin;
    int d1 = a->destination, d2 = b->destination;
    auto onedist = network->get_time(o1, d1);
    if (onedist)
    {
        double ratio = network->get_time(o1, o2) + network->get_time(o2, d1);
        ratio /= onedist;
        best = min(best, ratio);
    }
    auto twodist = network->get_time(o2, d2);
    if (twodist)
    {
        double ratio = network->get_time(o2, o1) + network->get_time(o1, d2);
        ratio /= twodist;
        best = min(best, ratio);
    }
    if (!onedist && !twodist)
        best = 0;
    return best;
}


struct rr_thread_data
{
    int time;
    map<Request*, set<Request*>>* rr_edges;
    Network const* network;
    vector<Request*> const * requests;
};


// The graph specifically checks for r1 boarding first.
void make_rrgraph(void* rr_data)
{
    struct thread_data* t = (struct thread_data*) rr_data;
    int start = t->start;
    int end = t->end;
    
    struct rr_thread_data* data = (struct rr_thread_data*) t->data;
    auto time = data->time;
    auto rr_edges = data->rr_edges;
    auto network = data->network;
    auto requests = data->requests;
    
    for (int i = start; i < end; i++)
    {
        Request* r1 = (*requests)[i];
        int start_node = r1->origin;
        vector<Request*> compatible_requests;
        std::map<const Request*,double> cost_ratio;
        
        for (Request* r2 : *requests)
        {
            if (*r1 == *r2)  // Don't pair with itself!
                continue;
            if (DISABLE_DIRECT_TRIPS && r1->original_req_id == -1)  // Ignore direct trips
                break;
            if (DISABLE_DIRECT_TRIPS && r2->original_req_id == -1)  // Ignore direct trips
                continue;
            // avoid pairing different multi-modal combination of the same request
            if (r1->original_req_id != -1 && r2->original_req_id != -1) {
                if (r1->original_req_id == r2->original_req_id && r1->bus_trip_id != r2->bus_trip_id)
                    continue;
            }
            // avoid pairing a first/last leg and the direct option of the same request
            if (r2->id == r1->original_req_id || r1->id == r2->original_req_id) {
                continue;
            }
            vector<Request*> request_list { r1 , r2 };
            
            // Heuristic to prune the requests without calling the travel function.
            int r2_origin = r2->origin;
            double buffer = 0;
            double min_wait = network->get_time(start_node, r2_origin) - buffer;
            if (min_wait + max(time, r1->entry_time) > r2->latest_boarding)
                continue;
            
            Vehicle dummyVehicle(0, 0, 4, start_node);

            pair<int,vector<NodeStop>> raw_path = routeplanner::travel(dummyVehicle, request_list, STANDARD,
                    *network, time);
            double direct_travel_cost = r1->ideal_traveltime + r2->ideal_traveltime;
            // if (raw_path.first >= 0 && raw_path.first <= MAX_ADD_COST*direct_travel_cost) // I.e., valid trip.
            if (raw_path.first >= 0) // I.e., valid trip.
            {
                compatible_requests.push_back(r2);
                cost_ratio[r2] = (double)raw_path.first / direct_travel_cost;
            }

        }
        
        auto sort_lambda = [network, &cost_ratio, r1](const Request* a, const Request* b) -> bool
        {
            double avalue = cost_ratio[a]; // detour_factor(r1, a, network);
            double bvalue = cost_ratio[b]; // detour_factor(r1, b, network);
            return avalue < bvalue;
        };
        sort(compatible_requests.begin(), compatible_requests.end(), sort_lambda);
        if (PRUNING_RR_K > 0 && compatible_requests.size() > PRUNING_RR_K) // Keep only the k best!
            compatible_requests.resize(PRUNING_RR_K);
        
        mtx.lock();
        (*rr_edges)[r1] = set<Request*>(compatible_requests.begin(), compatible_requests.end());
        mtx.unlock();
    }
}

// PNAS version : Performs sequence of steps, each in parallel, to produce RTV graph and then assignments. 
generator::assignment_result assignment(
        std::vector<Vehicle*> const & vehicles,
        std::vector<Request*> const & requests,
        std::map<Vehicle*, std::vector<Trip>> prev_trip_list,
        int time,
        Network const & network,
        Threads & threads)
{
    info("Building R-V edges of RV graph", Yellow);

    // Get previously assigned requests
    vector<int> prev_assigned_requests;
    std::map<int, double> time_to_pickup;
    for (auto v : vehicles) {
        for (auto r : v->pending_requests) {
            prev_assigned_requests.push_back(r->id);
            time_to_pickup[r->id] = network.get_vehicle_time(*v, r->origin);
        }
    }

    // Sort previously assigned requests by time to pickup.
    auto sort_lambda = [&time_to_pickup](int a, int b) -> bool
    {
        double avalue = time_to_pickup[a];
        double bvalue = time_to_pickup[b];
        return avalue < bvalue;
    };
    sort(prev_assigned_requests.begin(), prev_assigned_requests.end(), sort_lambda);

    // Pick first 20% requests from prev_assigned_requests
    size_t fixed_count = 0; //static_cast<size_t>(0.1 * prev_assigned_requests.size());
    // set<int> fixed_requests(prev_assigned_requests.begin(), prev_assigned_requests.begin() + fixed_count);
    set<int> fixed_requests;
    for (int r_id: prev_assigned_requests) {
        if (time_to_pickup[r_id] <= FIX_ASSIGNMENT_BEFORE) fixed_requests.insert(r_id); // Fix the requests close to being picked
    }

    int vr_edge_cnt = 0;
    map<Vehicle*, vector<Request*>> vr_edges;  // RV edges indexed by vehicle id.
    {
        map<Request*, vector<Vehicle*>> rv_edges;
        struct rv_thread_data rv_data {time, &rv_edges, &network, &requests, &vehicles};
        threads.auto_thread(requests.size(), make_rvgraph, (void*) &rv_data);
        rv_edges = prune_rvgraph(rv_edges, requests, vehicles, fixed_requests, time);

        for (auto x : rv_edges) // Invert the graph.
        {
            Request* r = x.first;
            vector<Vehicle*> vs = x.second;
            for (auto v : vs)
            {
                vr_edges[v].push_back(r);
                vr_edge_cnt += 1;
            }
        }
    }
    info("Total R-V edges of RV graph " + to_string(vr_edge_cnt), Yellow);
    
    info("Buidling R-R edges of RV graph", Yellow);
    map<Request*, set<Request*>> rr_edges;  // RR edges indexed by request id.
    {
        struct rr_thread_data rr_data {time, &rr_edges, &network, &requests};
        threads.auto_thread(requests.size(), make_rrgraph, (void*) &rr_data);
    }

    // stringstream rr; //ofstream rtv(RESULTS_DIRECTORY + "/rtv.log", ios_base::app); // Stringstream disables
    // rr << "TIME STAMP " << encode_time(time) << endl;
    // for (auto & x : rr_edges)
    // {
    //     Request* r = x.first;
    //     rr << "{'rid':" << r->id << ",'rr':[";
    //     for (auto & res : x.second)
    //     {
    //         rr << res->id << ",";
    //     }
    //     rr << "]}" << endl;
    // }
    // mtx.lock();
    // {
    //     ofstream rrfile(RESULTS_DIRECTORY + "/rr.log", ios_base::app);
    //     rrfile << rr.rdbuf();
    // }
    // mtx.unlock();

    int rr_count = 0;
    for (auto & x : rr_edges)
        rr_count += x.second.size();
    info("RR edges is of size " + to_string(rr_count), Red);
    
    info("Building RTV graph", Yellow);
    mtx.lock();
    {
        ofstream debuglog(RESULTS_DIRECTORY + "/debug.log", ios_base::app);
        debuglog << "TIME STAMP " << encode_time(time) << endl;
    }
    mtx.unlock();

    // Clean prev_trip_list by removing trips that have requests not in current requests
    set<int> valid_request_ids;
    for (auto r : requests) {
        valid_request_ids.insert(r->id);
    }
    
    for (auto & x : prev_trip_list)
    {
        Vehicle* v = x.first;
        vector<Trip> trips = x.second;
        vector<Trip> filtered_trips;
        
        for (auto & trip : trips) {
            bool all_requests_valid = true;
            for (auto req : trip.requests) {
                if (valid_request_ids.find(req->id) == valid_request_ids.end()) {
                    all_requests_valid = false;
                    break;
                }
            }
            if (all_requests_valid) {
                filtered_trips.push_back(trip);
            }
        }
        
        if (filtered_trips.empty()) {
            prev_trip_list.erase(v);
        } else {
            prev_trip_list[v] = filtered_trips;
        }
    }

    map<Vehicle*, vector<Trip>> trip_list;  // Store possible trips per vehicle
    {
        vector<Vehicle*> sorted_vs = vehicles;
        sort(sorted_vs.begin(), sorted_vs.end(),
                [vr_edges](Vehicle* & a, Vehicle* & b) -> bool {
                        if (vr_edges.count(a) && !vr_edges.count(b))
                            return true;
                        if (vr_edges.count(b) && !vr_edges.count(a))
                            return false;
                        if (vr_edges.count(a) && vr_edges.count(b))
                            if (vr_edges.at(a).size() > vr_edges.at(b).size())
                                return true;
                            else if (vr_edges.at(a).size() < vr_edges.at(b).size())
                                return false;
                        return a->id < b->id;
                });
        struct rtv_thread_data rtv_data {time, &rr_edges, &vr_edges, &trip_list, &network, &sorted_vs, &prev_trip_list};
        threads.mega_thread(vehicles.size(), make_rtvgraph, (void*) &rtv_data);
    }
    
    map<Vehicle*, vector<Trip>> linear_trip_list;
    for (auto & x : trip_list)
    {
        Vehicle* v = x.first;
        set<Request*> previous_assigned_passengers (v->pending_requests.begin(), v->pending_requests.end());
        vector<Trip> trips = x.second;
        for (auto & t : trips)
        {
            set<Request*> trip_requests = set<Request*>(t.requests.begin(), t.requests.end());
            if (std::includes(trip_requests.begin(), trip_requests.end(), 
                previous_assigned_passengers.begin(), previous_assigned_passengers.end()))
            {
                linear_trip_list[v].push_back(t);
            }
        }
    }

    int count = 0;
    for (auto & x : trip_list)
        count += x.second.size();
    info("Trip list is of size " + to_string(count), Red);
    
    { // Check to be sure no requests were downright rejected if they were previously assigned.
        set<Request*> rs;
        for (auto & x : trip_list)
            for (auto t : x.second)
                for (auto r : t.requests)
                    rs.insert(r);
        map<Request*,int> ps;
        for (auto v : vehicles)
            for (auto r : v->pending_requests)
                ps[r] = v->id;
        for (auto r : requests)
            if (r->assigned && !rs.count(r))
                if (!ps.count(r))
                    throw runtime_error("Help!  I was not included! " + to_string(r->id));
                else
                {
                    cout << r << " with id " << r->id << " on " << ps[r] << endl;
                    throw runtime_error("Help!  I was a pending request!");
                }
    }
    { // Check to be sure that all previous trips are included in the future possibilities!
        for (auto v : vehicles)
        {
            set<Request*> prev (v->pending_requests.begin(), v->pending_requests.end());
            bool found = false;
            for (auto & t : trip_list[v])
            {
                set<Request*> rs (t.requests.begin(), t.requests.end());
                if (prev == rs)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                // I have verified that the trip is feasible in rr and vr graph.  travel feasible too.
                cout << "Vid " << v->id << endl;
                throw runtime_error("Did not replicate the trip!");
            }
        }
    }

    // Output trace of generated trip_list.
    // stringstream rtv; //ofstream rtv(RESULTS_DIRECTORY + "/rtv.log", ios_base::app); // Stringstream disables
    // rtv << "TIME STAMP " << encode_time(time) << endl;
    // for (auto & x : trip_list)
    // {
    //     int vid = x.first->id;
    //     for (auto & t : x.second)
    //     {
    //         rtv << "{'v':" << vid << ",'rs':[";
    //         for (auto r : t.requests)
    //             rtv << r->id << ",";
    //         rtv << "],'c':" << t.cost << "}" << endl;
    //     }
    // }
    // mtx.lock();
    // {
    //     ofstream rtvfile(RESULTS_DIRECTORY + "/rtv.log", ios_base::app);
    //     rtvfile << rtv.rdbuf();
    // }
    // mtx.unlock();

    map<Vehicle*,Trip> assignment;
    double time_limit = GUROBI_TIME_LIMIT;
    
    map<Vehicle*,Trip> empty_assignment;
    map<Vehicle*,Trip> linear_assignment = ilp_common_gurobi::ilp_assignment_gurobi(linear_trip_list, requests, time, 20, empty_assignment);
    
    {
        ofstream assignment_file(RESULTS_DIRECTORY + "/linear_assignment.log", std::ios_base::app);
        assignment_file << "TIME STAMP:" << encode_time(time) << endl;
        for (auto &t : linear_assignment) {
            assignment_file << "\tAssigned vid " << t.first->id << "\t";
            for (auto r : t.second.requests) {
                assignment_file << r->id << "\t";
            }
            assignment_file << endl;
        }
        empty_assignment = linear_assignment;
    }

    try { 
        assignment = ilp_common_gurobi::ilp_assignment_gurobi(trip_list, requests, time, time_limit, empty_assignment);
    } catch (GRBException e) {
        cout << "GRBException ocurred" << endl;
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
        assignment = empty_assignment;
    }

    {
        ofstream unassigned_file(RESULTS_DIRECTORY + "/unassigned_requests.log", std::ios_base::app);
        unassigned_file << "TIME STAMP:" << encode_time(time) << endl;
        set<int> assigned_requests_ids;
        for (const auto& pair : assignment) {
            for (const auto& req : pair.second.requests) {
                if (req->original_req_id != -1)
                    assigned_requests_ids.insert(req->original_req_id);
                else
                    assigned_requests_ids.insert(req->id);
            }
        }
        for (const auto& req : requests) {
            if (req->original_req_id == -1 && assigned_requests_ids.count(req->id) == 0) {
                unassigned_file << "Unassigned request id: " << req->id << endl;
            }
        }
    }

    // Update rolling performance statistics for the next iteration's pruning.
    {
        // New requests = those that entered this iteration (not previously assigned).
        set<int> new_req_ids;
        for (auto r : requests)
            if (!r->assigned && r->entry_time >= time - INTERVAL)
                new_req_ids.insert(r->id);

        // Collect all request IDs that appear in non-fake assigned trips.
        set<int> served_ids;
        for (auto& kv : assignment)
            if (!kv.second.is_fake)
                for (auto r : kv.second.requests)
                    served_ids.insert(r->id);

        int n_served_new = 0;
        for (int rid : new_req_ids)
            if (served_ids.count(rid)) n_served_new++;

        double p_served = new_req_ids.empty()
            ? 1.0
            : (double)n_served_new / new_req_ids.size();

        int n_assigned_trips = 0;
        for (auto& kv : assignment)
            if (!kv.second.requests.empty() && !kv.second.is_fake)
                n_assigned_trips++;
        double trips_per_veh = vehicles.empty()
            ? 0.0
            : (double)n_assigned_trips / vehicles.size();

        stat_n_served.push_back(n_served_new);
        stat_p_served.push_back(p_served);
        stat_trips_per_veh.push_back(trips_per_veh);
        if ((int)stat_n_served.size()      > STAT_HISTORY) stat_n_served.pop_front();
        if ((int)stat_p_served.size()      > STAT_HISTORY) stat_p_served.pop_front();
        if ((int)stat_trips_per_veh.size() > STAT_HISTORY) stat_trips_per_veh.pop_front();
    }

    struct generator::assignment_result ass_result {trip_list, assignment};
    return ass_result;
}

}

