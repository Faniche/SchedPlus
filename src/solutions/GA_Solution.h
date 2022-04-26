//
// Created by faniche on 22-4-20.
//

#ifndef SCHEDPLUS_GA_SOLUTION_H
#define SCHEDPLUS_GA_SOLUTION_H

#include <vector>
#include "../components/Flow.h"
#include "../../lib/openGA.hpp"

struct TTFlows {
    std::vector<uint32_t> offsets;
    std::vector<uint32_t> selected_route_idx;
    std::map<uint32_t, std::vector<std::pair<uint32_t , uint32_t >>> transmit_intervals;
    std::string to_string(std::ostringstream &oss) const {
        oss << "{" << offsets[0];
        for (int i = 1; i < offsets.size(); ++i) {
            oss << ", " << offsets[i];
        };
        oss << "}";
        return oss.str();
    }
};

struct MyMiddleCost {
    // This is where the results of simulation
    // is stored but not yet finalized.
    std::vector<uint32_t> delivery_guarantees;

    double e2e;

    double ddl;

//    double delayQueue = 0;
//
//    double bandwidthUsage = 0;

};

typedef EA::Genetic<TTFlows, MyMiddleCost> GA_Type;
typedef EA::GenerationType<TTFlows, MyMiddleCost> Generation_Type;



#endif //SCHEDPLUS_GA_SOLUTION_H
