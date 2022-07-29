//
// Created by faniche on 22-4-20.
//

#ifndef SCHEDPLUS_GA_SOLUTION_H
#define SCHEDPLUS_GA_SOLUTION_H

#include <vector>
#include "../components/Flow.h"
#include "../../lib/openGA/openGA.hpp"

using std::vector;
using std::map;

struct TTFlows {
    vector<uint64_t> offsets;
    vector<uint64_t> selected_route_idx;
};

struct MyMiddleCost {

    /*   flow_id       hop     offset     */
    map<uint32_t, map<uint8_t, uint64_t>> p6_traffic_offsets;
/*   flow_id       hop     offset     */
    map<uint32_t, map<uint8_t, uint64_t>> traffic_offsets;
    map<uint32_t, uint64_t> ddl_or_e2e;
    /*   flow_id     snd_times       hop     offset     */
//    map<uint32_t, map<uint64_t, map<uint8_t, uint64_t>>> p5_traffic_offsets;

    /*   flow_id     hop           snd_times offset     */
    map<uint32_t, map<uint8_t, map<uint64_t, uint64_t>>> p5_traffic_offsets;

    /*   flow_id     snd_times   e2e  */
    map<uint32_t, vector<std::pair<uint64_t, uint64_t>>> p5_e2e;

    /*   flow_id  jitter  */
    map<uint32_t, double> p5_cached_jitter;

    /*  link_id                    flow_id   hop */
    map<uint32_t, vector<std::pair<uint32_t, uint8_t>>> link_flows;

    vector<uint32_t> cached_flows;

    vector<uint32_t> uncached_flows;

    /*  link_id   hyperperiod of link */
    map<uint32_t, uint64_t> link_hyperperiod;

    map<uint32_t, uint64_t> link_gcl_size;

    map<uint32_t, uint64_t> link_gcl_merge_count;

    // the minimum variance of generation
    double variance = 0;

    double e2e;

    double ddl;

    double total_transmit;

    double v_transmit;

    double total_cache;

    double total_gcl;
//    double delayQueue = 0;
//
//    double bandwidthUsage = 0;

};

typedef EA::Genetic<TTFlows, MyMiddleCost> GA_Type;
typedef EA::GenerationType<TTFlows, MyMiddleCost> Generation_Type;



#endif //SCHEDPLUS_GA_SOLUTION_H
