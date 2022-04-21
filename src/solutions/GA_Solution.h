//
// Created by faniche on 22-4-20.
//

#ifndef SCHEDPLUS_GA_SOLUTION_H
#define SCHEDPLUS_GA_SOLUTION_H

#include <vector>
#include "../components/Flow.h"
#include "../../lib/openGA.hpp"

typedef size_t node_idx;

struct TTFlows {
    std::vector<std::reference_wrapper<Flow>> flows;
//    std::map<node_idx, Node *> nodeMap;
    std::map<node_idx, std::vector<std::reference_wrapper<Flow>>> flowGroup;

    std::string to_string(std::ostringstream &oss) const {
        oss << "{" << flows[0].get().getOffset();
        for (auto flow: flows) {
            oss << ", " << flow.get().getOffset();
        };
        oss << "}";
        return oss.str();
    }
};

struct MyMiddleCost {
    // This is where the results of simulation
    // is stored but not yet finalized.

    double e2e;

    double ddl;

    double delayQueue = 0;

    double bandwidthUsage = 0;

};

typedef EA::Genetic<TTFlows, MyMiddleCost> GA_Type;
typedef EA::GenerationType<TTFlows, MyMiddleCost> Generation_Type;



#endif //SCHEDPLUS_GA_SOLUTION_H
