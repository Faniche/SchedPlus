//
// Created by faniche on 22-6-22.
//

#ifndef SCHEDPLUS_SAVESOLUTION_H
#define SCHEDPLUS_SAVESOLUTION_H

#include <string>
#include <cstdint>
#include <vector>
#include <map>
#include <fstream>
#include <spdlog/spdlog.h>
#include "../components/Node.h"
#include "../components/Link.h"
#include "../components/Flow.h"
#include "../event/Event.h"
#include "../Utils.h"
#include "../../lib/pugixml/pugixml.hpp"
#include "GA_Solution.h"

using std::vector;
using std::map;

class SaveSolution {
public:
    uint64_t hyperPeriod;
    vector<schedplus::Event> events;
    map<uint32_t, uint64_t> link_hyperperiod;

    SaveSolution(uint64_t hyperPeriod, const vector<schedplus::Event> &events);

    static void save_route(const TTFlows &p, MyMiddleCost &c,
                    const std::string &route_file_location,
                    vector<Node *> &nodes,
                    vector<Node *> &swList,
                    vector<std::reference_wrapper<DirectedLink>> &links);

    void saveGCL(const std::string &gcl_file_location,
                 vector<std::reference_wrapper<Flow>> flows,
                 map<node_idx, Node *> nodeMap,
                 vector<Node *> &swList,
                 vector<std::reference_wrapper<DirectedLink>> &links);

    void saveSwPortSchedule(const std::string &sched_file_location,
                            vector<Node *> &swList,
                            vector<std::reference_wrapper<DirectedLink>> &links);

    void saveEsSchedule(const std::string &sched_file_location,
                        const vector<std::reference_wrapper<Flow>>& flows,
                        map<node_idx, Node *> nodeMap,
                        vector<std::reference_wrapper<DirectedLink>> &links);

    static void saveGCL(const TTFlows &p, MyMiddleCost &c,
                 const vector<std::reference_wrapper<Flow>>& flows,
                 vector<std::reference_wrapper<DirectedLink>> &links);

    void saveEvent(const TTFlows &p, MyMiddleCost &c,
                   const vector<std::reference_wrapper<Flow>>& flows,
                   const std::string& event_file);

    void saveIni(const std::string &route_file,
                 const std::string &gcl_file,
                 const std::string &ini_file,
                 const std::string &ned_file,
                 vector<Node *> &swList,
                 vector<Node *> &esList,
                 const vector<std::reference_wrapper<Flow>>& flows,
                 uint64_t solution_id) const;
};


#endif //SCHEDPLUS_SAVESOLUTION_H
