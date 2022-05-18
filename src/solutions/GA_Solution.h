//
// Created by faniche on 22-4-20.
//

#ifndef SCHEDPLUS_GA_SOLUTION_H
#define SCHEDPLUS_GA_SOLUTION_H

#include <vector>
#include "../components/Flow.h"
#include "../../lib/openGA.hpp"

struct TTFlows {
    std::vector<uint64_t> offsets;
    std::vector<uint64_t> selected_route_idx;
};

struct MyMiddleCost {
    // This is where the results of simulation
    // is stored but not yet finalized.

    //       link_id                          start     interval
    std::map<uint32_t , std::vector<std::pair<uint64_t , uint64_t>>> transmit_intervals;
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
