//
// Created by faniche on 22-4-20.
//

#ifndef SCHEDPLUS_GA_SOLUTION_H
#define SCHEDPLUS_GA_SOLUTION_H

#include <vector>
#include "../components/Flow.h"
#include "../../lib/openGA.hpp"

using std::vector;
using std::map;

struct TTFlows {
    vector<uint64_t> offsets;
    vector<uint64_t> selected_route_idx;
};

struct MyMiddleCost {

    /*   flow_id     snd_times       hop     offset     */
    map<uint32_t, map<uint64_t , map<uint8_t, uint64_t>>> p6_traffic_offsets;

    /*   flow_id     snd_times       hop     offset     */
    map<uint32_t, map<uint64_t, map<uint8_t, uint64_t>>> p5_traffic_offsets;

    /*   flow_id     snd_times   e2e  */
    map<uint32_t, map<uint64_t, uint64_t>> p5_e2e;

    map<uint32_t, uint64_t> link_hyperperiod;
//    map<uint32_t, vector<map<uint32_t ,vector<map<uint64_t ,vector<map<uint32_t ,vector<map<uint32_t ,vector<uint64_t >>>>>>>>>> flow_collision;

    // the minimum variance of generation
    double variance = 0;

    double e2e;

    double ddl;

//    double delayQueue = 0;
//
//    double bandwidthUsage = 0;

};

typedef EA::Genetic<TTFlows, MyMiddleCost> GA_Type;
typedef EA::GenerationType<TTFlows, MyMiddleCost> Generation_Type;



#endif //SCHEDPLUS_GA_SOLUTION_H
