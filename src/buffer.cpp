/*
 * The MIT License
 *
 * Copyright 2018 Vindula Jayawardana and Matthew Zalesak.
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

#include "buffer.hpp"
#include "formatting.hpp"
#include "settings.hpp"

#include <set>
#include <map>

#define MAX_STAY_TIME 24 //24 hours
using namespace std;

vector<Vehicle*> buffer::get_active_vehicles(vector<Vehicle> & vehicles, int time)
{
    vector<Vehicle*> buffer;
    for (auto & v : vehicles)
        buffer.push_back(&v);
    
    return buffer;
}

pair<vector<Request*>, int> buffer::get_new_requests(vector<Request> & requests, vector<Request> & leg_requests, int time)
{
    multimap<int,Request*> entry_times;
    for (auto & r : requests)
    {
        if (r.entry_time <= time && time < r.entry_time + INTERVAL) // If already entered, but not too long ago.
        {
            entry_times.insert(make_pair(r.entry_time, &r));
        }
    }
    vector<Request*> buffer;

    int count = 0;
    int last_entry_time = time;
    for (auto &x : entry_times)
    {
        Request* r = x.second;
        count ++;
        buffer.push_back(r);

        // Group leg_requests for this original request by bus_trip_id.
        // When ONLY_ALLOW_SINGLE_LEG is true, skip any bus_trip_id group that
        // has both a first-leg (leg_type=0) and a last-leg (leg_type=1).
        if (ONLY_ALLOW_SINGLE_LEG)
        {
            map<int, vector<Request*>> by_trip;
            for (auto & l_r : leg_requests)
                if (l_r.original_req_id == r->id)
                    by_trip[l_r.bus_trip_id].push_back(&l_r);

            for (auto & kv : by_trip)
            {
                bool has_first = false, has_last = false;
                for (auto * lr : kv.second)
                {
                    if (lr->leg_type == 0) has_first = true;
                    if (lr->leg_type == 1) has_last  = true;
                }
                if (has_first && has_last)
                    continue; // skip combined pair
                for (auto * lr : kv.second)
                    buffer.push_back(lr);
            }
        }
        else
        {
            for (auto & l_r : leg_requests)
                if (l_r.original_req_id == r->id)
                    buffer.push_back(&l_r);
        }
        if (count >= MAX_REQ_PER_ITER && last_entry_time < r->entry_time) break;
        last_entry_time = r->entry_time;
    }
    last_entry_time += INTERVAL;
    info("Count of new requests: "+ to_string(count), Purple);
    info("last_entry_time: "+ to_string(last_entry_time), Purple);
    if (count < MAX_REQ_PER_ITER) last_entry_time = time + INTERVAL;

    return make_pair(buffer, last_entry_time);
}

