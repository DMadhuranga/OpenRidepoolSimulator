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
#include <mutex> // <-- Guilty party.  Secretly includes "chrono"
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace std;
 
namespace ilp_full
{

mutex mtx;

struct rtv_thread_data
{
    int time;
    map<Request*, set<Request*>>* rr_edges;
    map<Vehicle*, vector<Request*>>* vr_edges;
    map<Vehicle*, vector<Trip>>* trip_list;
    Network const* network;
    vector<Vehicle*> const* vehicles;
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
        
        mtx.lock();
        set<Request*> initial_pairing ((*vr_edges)[v].begin(), (*vr_edges)[v].end());
        mtx.unlock();
        
        round.push_back(vector<Trip>());
        initial_pairing.insert(v->pending_requests.begin(), v->pending_requests.end());
        mtx.lock();
        if (initial_pairing.size() > (*vr_edges)[v].size())
            cout << "Added "; 
            for (auto r : initial_pairing)
            {
                if (find((*vr_edges)[v].begin(), (*vr_edges)[v].end(), r) == (*vr_edges)[v].end())
                    cout << r->id << ",";
            }
            cout << " reqs to vid" << v->id << endl;
        mtx.unlock();
        for (auto r : initial_pairing)
        {
            vector<Request*> requests {r};
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
        outputs << endl;
        
        int existing_trip_size = previous_assigned_passengers.size();
        outputs << "Number of assigned passengers: " << existing_trip_size << endl;
        outputs << "Number of passengers on board: " << v->passengers.size() << endl;
        // In all subsequent rounds, take pairs from the previous round and build if they add one new element.
        int counter = 0;
        while (round.size() <= existing_trip_size + 1 || (round[round.size() - 1].size() && !timeout))
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
                    throw runtime_error("Previous assignment no longer feasible. Vid: "+to_string(v->id));
                }
                round[k].push_back(previoustrip);
            }

            if (DISABLE_REASSIGNMENT && k <= existing_trip_size)
                continue;

            // outputs << "sizeof k-1: " << to_string(round[k - 1].size()) << endl;
            for (auto first = 0; first < round[k - 1].size(); first++)
            {
                // always allow to build on top of previous assignment.
                if (timeout && (k != existing_trip_size+1 || first > 0))
                    break;
                // Get new request set.
                set<Request*> left (round[k-1][first].requests.begin(), round[k-1][first].requests.end());
                // if (k > existing_trip_size)
                // {
                //     for (auto r : left)
                //         outputs << r->id << ",";
                //     outputs << "left" << endl;
                // }
                int prev_round = k - 1;
                if (k == existing_trip_size+1 && (DISABLE_REASSIGNMENT || timeout))
                    prev_round = 1;
                for (auto second = first + 1; second < round[prev_round].size(); second++)
                {
                    // Check the time.
                    auto end_time = chrono::steady_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds> (end_time - start_time).count();
                    if ((k != existing_trip_size+1 || first > 0) && RTV_TIMELIMIT && duration > RTV_TIMELIMIT)
                    {
                        outputs << "timeout" << endl;
                        timeout = true;
                        break;
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
                    if (k != existing_trip_size + 1 && first > 0)
                    {
                        auto end_time = chrono::steady_clock::now();
                        auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
                        preokay = (duration <= RTV_TIMELIMIT);
                    }
                    pair<int,vector<NodeStop>> path;
                    if (k == existing_trip_size + 1 && first == 0){
                        path = routeplanner::travel(
                                *v, request_vector, STANDARD, *network, time);
                    } else {
                        path = routeplanner::time_travel(
                                *v, request_vector, STANDARD, *network, time, start_time);
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

            outputs << "Trip size: " << k << ", number of trips: " << round[k].size() << endl;
        }
        
        // Convert into appropriate format.  This include adding in the pending requests.
        vector<Trip> potential_trip_list;
        for (auto & list : round)
            potential_trip_list.insert(potential_trip_list.end(), list.begin(), list.end());
        
        for (auto & t : potential_trip_list)
            if (t.cost == -1)
                throw runtime_error("Negative cost not cleaned up.");

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

        // outputs << "Ended RTV for vid " << v->id << endl;
        // mtx.lock();
        // {
        //     ofstream debuglogfile (RESULTS_DIRECTORY + "/debug.log", std::ios_base::app);
        //     debuglogfile << outputs.rdbuf();
        // }
        // mtx.unlock();

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
        vector<Request*> requests { r };
        int origin = r->origin;
        double buffer = 0;
        vector<Vehicle*> compatible_vehicles;
        stringstream outputs;
        outputs << "Starting RV for rid " << r->id << endl;

        multimap<int,Vehicle*> nearest_vs;
        if (!DISABLE_DIRECT_TRIPS || r->original_req_id != -1)  // Ignore direct trips
        {
            for (Vehicle* v : *vehicles)
            {
                double min_wait = network->get_vehicle_time(*v, origin) - buffer;
                if (time + min_wait > r->latest_boarding) continue;
                nearest_vs.insert(make_pair(min_wait, v));
            }
        }

        outputs << "Size of nearest_vs: " << nearest_vs.size() << endl;
        
        int time_to_pickup = r->latest_boarding - time;
        outputs << "Time to pickup: " << time_to_pickup << endl;
        int count = 0;
        if (!r->assigned) {    
            for (auto &x : nearest_vs)
            {
                Vehicle* v = x.second;
                pair<int,vector<NodeStop>> raw_path = routeplanner::travel(*v, requests, STANDARD, *network, time);
                if (raw_path.first >=0 )
                {
                    compatible_vehicles.push_back(v);
                    // if (PRUNING_RV_K > 0 && ++count >= PRUNING_RV_K) break;
                }
            }
        } else {
            std::map<const Vehicle*,double> cost_ratio;

            for (auto &x : nearest_vs)
            {
                Vehicle* v = x.second;
                pair<int,vector<NodeStop>> raw_path = routeplanner::travel(*v, requests, STANDARD, *network, time);
                if (raw_path.first >=0 )
                {
                    compatible_vehicles.push_back(v);
                    cost_ratio[v] = (double)raw_path.first/(r->ideal_traveltime);
                    if (PRUNING_RV_K > 0 && ++count >= PRUNING_RV_K/2 && time_to_pickup <= 300) break;
                }
            }

            auto sort_lambda = [&cost_ratio](const Vehicle* a, const Vehicle* b) -> bool
            {
                double avalue = cost_ratio[a]; // detour_factor(r1, a, network);
                double bvalue = cost_ratio[b]; // detour_factor(r1, b, network);
                return avalue < bvalue;
            };
            sort(compatible_vehicles.begin(), compatible_vehicles.end(), sort_lambda);
            if (PRUNING_RV_K > 0 && compatible_vehicles.size() > PRUNING_RV_K) // Keep only the k best!
                compatible_vehicles.resize(PRUNING_RV_K);
        }

        outputs << "Size of compatible_vehicles: " << compatible_vehicles.size() << endl;
        outputs << "Ended RV for rid " << r->id << endl;
        // mtx.lock();
        // {
        //     ofstream debuglogfile (RESULTS_DIRECTORY + "/debug.log", std::ios_base::app);
        //     debuglogfile << outputs.rdbuf();
        // }
        // mtx.unlock();
        
        
        mtx.lock();
        (*rv_edges)[r] = compatible_vehicles;
        mtx.unlock();
    }
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
std::map<Vehicle*, Trip> assignment(
        std::vector<Vehicle*> const & vehicles,
        std::vector<Request*> const & requests,
        int time,
        Network const & network,
        Threads & threads)
{
    info("Building R-V edges of RV graph", Yellow);
    int vr_edge_cnt = 0;
    map<Vehicle*, vector<Request*>> vr_edges;  // RV edges indexed by vehicle id.
    {
        map<Request*, vector<Vehicle*>> rv_edges;
        struct rv_thread_data rv_data {time, &rv_edges, &network, &requests, &vehicles};
        threads.auto_thread(requests.size(), make_rvgraph, (void*) &rv_data);
        
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
        struct rtv_thread_data rtv_data {time, &rr_edges, &vr_edges, &trip_list, &network, &sorted_vs};
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
                    throw runtime_error("Help!  I was not included!");
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
    }

    try { 
        assignment = ilp_common_gurobi::ilp_assignment_gurobi(trip_list, requests, time, time_limit, linear_assignment);
    } catch (GRBException e) {
        cout << "GRBException ocurred" << endl;
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
        assignment = linear_assignment;
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

    return assignment;
}

}

