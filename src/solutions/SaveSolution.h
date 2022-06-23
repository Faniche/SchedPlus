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
#include "../../lib/pugixml.hpp"

using std::vector;
using std::map;

class SaveSolution {
public:
    vector<Node *> nodes;
    vector<Node *> esList;
    vector<Node *> swList;
    map<node_idx, Node *> nodeMap;
    vector<std::reference_wrapper<DirectedLink>> links;
    vector<std::reference_wrapper<Flow>> flows;
    std::string output_location;
    uint64_t hyperPeriod;
    vector<schedplus::Event> events;
    map<uint32_t, uint64_t> link_hyperperiod;

    SaveSolution(const vector<Node *> &nodes, const vector<Node *> &esList, const vector<Node *> &swList,
                 const map<node_idx, Node *> &nodeMap, const vector<std::reference_wrapper<DirectedLink>> &links,
                 const vector<std::reference_wrapper<Flow>> &flows, const std::string &outputLocation,
                 uint64_t hyperPeriod, const vector<schedplus::Event> &events,
                 const map<uint32_t, uint64_t> &linkHyperperiod);

    void save_route(const std::string &route_file_location,
                    map<uint32_t, vector<uint32_t>> &link_flows);

    void saveGCL(const std::string &gcl_file_location, const vector<uint64_t> &selected_route_idx);

    void saveSwPortSchedule(const std::string &sched_file_location, const vector<uint64_t> &selected_route_idx);

    void saveEsSchedule(const std::string &sched_file_location, const vector<uint64_t> &selected_route_idx);

    void saveGCL(const vector<uint64_t> &offsets,
                 const vector<uint64_t> &selected_route_idx);

    void saveEvent(const vector<uint64_t> &offsets,
                   const vector<uint64_t> &selected_route_idx,
                   const std::string &event_file_location);

    void saveIni(const std::string &route_file,
                 const std::string &gcl_file,
                 const std::string &ini_file,
                 const std::string &ned_file,
                 uint64_t solution_id);
};


#endif //SCHEDPLUS_SAVESOLUTION_H
