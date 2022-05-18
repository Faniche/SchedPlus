//
// Created by faniche on 22-4-20.
//

#ifndef SCHEDPLUS_MYFUNCTIONS_H
#define SCHEDPLUS_MYFUNCTIONS_H


#include <string>
#include <iostream>
#include <fstream>
#include <functional>
#include <utility>
#include <iomanip>
#include "GA_Solution.h"
#include "../Utils.h"
#include "../../lib/openGA.hpp"

class MyFunctions {
private:
    std::vector<Node *> nodes;
    std::vector<Node *> esList;
    std::vector<Node *> swList;
    std::map<node_idx, Node *> nodeMap;
    std::vector<std::reference_wrapper<DirectedLink>> links;
    std::vector<std::reference_wrapper<Flow>> flows;
    std::map<node_idx, std::vector<std::reference_wrapper<Flow>>> flowGroup;

    static bool compIntervals(const std::pair<uint64_t, uint64_t> &p1, const std::pair<uint64_t, uint64_t> &p2) {
        return p1.first < p2.first;
    }

    static std::map<uint32_t, uint64_t> getLinkHyperperiods(const std::vector<std::reference_wrapper<Flow>> &flows,
                                                            const std::vector<uint64_t> &selected_route_idx) {
        std::map<uint32_t, uint64_t> link_hyperperiod;
        /* Calculate all hyperperiod of links */
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow.get().getId() - 1]].getLinks()) {
                uint32_t link_id = link.get().getId();
                if (link_hyperperiod.contains(link_id)) {
                    link_hyperperiod[link_id] = Util::lcm(link_hyperperiod[link_id], flow.get().getPeriod());
                } else {
                    link_hyperperiod[link_id] = flow.get().getPeriod();
                }
            }
        }
        return link_hyperperiod;
    }

    static void setTimeIntervals(const std::vector<std::reference_wrapper<Flow>> &flows,
                                 const std::vector<uint64_t> &offsets,
                                 const std::vector<uint64_t> &selected_route_idx,
                                 std::map<uint32_t, std::vector<std::pair<uint64_t, uint64_t>>> &transmit_intervals) {
        //        link_id     mem
        std::map<uint32_t, uint64_t> mem_allocate;
        //       link_id   src_port_hyperperiod
        std::map<uint32_t, uint64_t> link_hyperperiod;

        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow.get().getId() - 1]].getLinks()) {
                uint32_t link_id = link.get().getId();
                if (link_hyperperiod.contains(link_id)) {
                    link_hyperperiod[link_id] = Util::lcm(link_hyperperiod[link_id], flow.get().getPeriod());
                } else {
                    link_hyperperiod[link_id] = flow.get().getPeriod();
                }
            }
        }

        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow.get().getId() - 1]].getLinks()) {
                uint32_t link_id = link.get().getId();
                uint64_t sendTimes = link_hyperperiod[link_id] / flow.get().getPeriod();
                for (int i = 0; i < sendTimes; ++i) {
                    if (mem_allocate.contains(link_id)) {
                        mem_allocate[link_id] += sendTimes;
                    } else {
                        mem_allocate[link_id] = sendTimes;
                    }
                }
            }
        }
        for (auto &[link_id, gcl_entities]: transmit_intervals) {
            gcl_entities.reserve(mem_allocate[link_id]);
        }

        for (auto const &flow: flows) {
            /* Calculate GCL */
            uint32_t flow_id = flow.get().getId() - 1;
            uint64_t offset = offsets[flow_id];
            uint64_t accumulatedDelay = offset;
            uint64_t prop_delay = 0, trans_delay = 0, proc_delay = 0;
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow_id]].getLinks()) {
                uint32_t link_id = link.get().getId();
                proc_delay = link.get().getSrcNode()->getDpr();
                trans_delay = flow.get().getFrameLength() * link.get().getSrcPort().getMacrotick();
                accumulatedDelay += proc_delay;
                uint64_t sendTimes = link_hyperperiod[link_id] / flow.get().getPeriod();
                for (uint32_t i = 0; i < sendTimes; ++i) {
                    uint64_t start_time = accumulatedDelay + i * flow.get().getPeriod();
                    transmit_intervals[link_id].emplace_back(std::pair(start_time, trans_delay));
                }
                accumulatedDelay += trans_delay;
                prop_delay = link.get().getLen() * link.get().getPropSpeed();
                accumulatedDelay += prop_delay;
            }
        }
        for (auto &[link_idx, intervals]: transmit_intervals) {
            std::sort(intervals.begin(), intervals.end(), compIntervals);
        }
    }


public:
    std::ofstream output_file;

    MyFunctions(const std::string &output_location,
                std::vector<Node *> nodes,
                std::vector<Node *> esList,
                std::vector<Node *> swList,
                std::map<node_idx, Node *> nodeMap,
                std::vector<DirectedLink> &_links,
                std::vector<Flow> &_flows,
                std::map<node_idx, std::vector<std::reference_wrapper<Flow>>> flowGroup) :
            nodes(std::move(nodes)),
            esList(std::move(esList)),
            swList(std::move(swList)),
            nodeMap(std::move(nodeMap)),
            flowGroup(std::move(flowGroup)) {
//        output_file.open(output_location);
        links.assign(_links.begin(), _links.end());
        flows.assign(_flows.begin(), _flows.end());
    }

    ~MyFunctions() {
        output_file.close();
    }

    void init_genes(TTFlows &p, const std::function<double(void)> &rnd01) {
        p.offsets.assign(flows.size(), 0);
        p.selected_route_idx.assign(flows.size(), 0);
        /* Initialize the offset and route index of flow. */
        for (auto const &flow: flows) {
            uint32_t flow_id = flow.get().getId() - 1;
            /* Transmit delay of frame, unit: ns */
            //      delay     =  frame_length   *    port_macrotick
            int srcTransDelay =
                    flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
            uint64_t offset = (flow.get().getPeriod() - srcTransDelay) * rnd01();
            p.offsets[flow_id] = offset;
            if (flow.get().getRoutes().size() == 1) {
                p.selected_route_idx[flow_id] = 0;
            } else {
                uint64_t route_idx = (flow.get().getRoutes().size()) * rnd01();
                p.selected_route_idx[flow_id] = route_idx;
            }

        }
    }

    bool eval_solution(const TTFlows &p, MyMiddleCost &c) {
        /* Check delivery guarantees. */
        uint64_t max_ddl = 0, max_e2e = 0;
        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId() - 1;
            uint64_t e2e = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getE2E();
            /* Check delivery guarantees */
            if (flow.get().getPriorityCodePoint() == P6) {
                uint64_t ddl = p.offsets[flow_id] + e2e;
                if (ddl > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                    return false;
                /* the max ddl in current solution */
                if (ddl > max_ddl)
                    max_ddl = ddl;
            } else if (flow.get().getPriorityCodePoint() == P5) {
                if (e2e > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                    return false;
                /* the max e2e in current solution */
                if (e2e > max_e2e)
                    max_e2e = e2e;
            }
        }

        /* Check gcl collision */
        //       link_id                          start     interval
        std::map<uint32_t, std::vector<std::pair<uint64_t, uint64_t>>> transmit_intervals;
        setTimeIntervals(flows, p.offsets, p.selected_route_idx, transmit_intervals);
        for (auto &[link_idx, intervals]: transmit_intervals)
            for (int i = 0; i < intervals.size() - 1; ++i)
                if (intervals[i].first + intervals[i].second > intervals[i + 1].first)
                    return false;


        /* Get variance of current solution */
        std::vector<uint64_t> link_gcl_size(transmit_intervals.size());
        for (auto &[link_idx, intervals]: transmit_intervals)
            link_gcl_size.emplace_back(intervals.size());
        double mean = std::accumulate(link_gcl_size.begin(), link_gcl_size.end(), 0.0) / link_gcl_size.size();
        std::vector<double> diff(link_gcl_size.size());
        std::transform(link_gcl_size.begin(), link_gcl_size.end(), diff.begin(), [mean](double x) {
            return x - mean;
        });
        c.variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / link_gcl_size.size();

        /* set ddl and e2e for solution */
        c.ddl = max_ddl;
        c.e2e = max_e2e;

        spdlog::set_level(spdlog::level::info);
        spdlog::get("console")->debug("*******Correct solution*******");
        spdlog::get("console")->debug("\tlink\troute\toffset");
        std::for_each(flows.begin(), flows.end(),
                      [&](std::reference_wrapper<Flow> flow) {
                          spdlog::get("console")->debug("\t{}\t{}\t{}", flow.get().getId() - 1,
                                                        p.selected_route_idx[flow.get().getId() - 1],
                                                        p.offsets[flow.get().getId() - 1]);
                      });
        spdlog::get("console")->debug("******************************");
        return true; // solution is accepted
    }

    TTFlows mutate(
            const TTFlows &X_base,
            const std::function<double(void)> &rnd01,
            double shrink_scale) {
        TTFlows X_new;
        const double mu = 0.2 * shrink_scale; // mutation radius (adjustable)
        X_new = X_base;
        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId() - 1;
            uint64_t offset = 0, route = 0;
            bool in_range;
            do {
                in_range = true;
                offset = X_base.offsets[flow_id] + mu * (rnd01() - rnd01());
                int srcTransDelay =
                        flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
                in_range = in_range && offset >= 0 && offset < flow.get().getPeriod() - srcTransDelay;
                route = X_base.selected_route_idx[flow_id] + mu * (rnd01() - rnd01());
                in_range = in_range && route >= 0 && route < flow.get().getRoutes().size();
            } while (!in_range);
            X_new.offsets[flow_id] = offset;
            X_new.selected_route_idx[flow_id] = route;
        }
        return X_new;
    }

    TTFlows crossover(
            const TTFlows &X1,
            const TTFlows &X2,
            const std::function<double(void)> &rnd01) {
        TTFlows X_new;
        X_new = X1;
        double r_offset = rnd01();
        double r_route = rnd01();
        for (int i = 0; i < X1.offsets.size(); ++i) {
            uint64_t offset = r_offset * X1.offsets[i] + (1.0 - r_offset) * X2.offsets[i];
            X_new.offsets[i] = offset;
            uint64_t route = r_route * X1.selected_route_idx[i] + (1.0 - r_route) * X2.selected_route_idx[i];
            X_new.selected_route_idx[i] = route;
        }
        return X_new;
    }

    std::vector<double> calculate_MO_objectives(const GA_Type::thisChromosomeType &X) {
        return {
                X.middle_costs.variance,
                X.middle_costs.e2e,
                X.middle_costs.ddl
        };
    }

    void MO_report_generation(
            int generation_number,
            const EA::GenerationType<TTFlows, MyMiddleCost> &last_generation,
            const std::vector<unsigned int> &pareto_front) {
        (void) last_generation;

        std::cout << "Generation [" << generation_number << "], ";
        std::cout << "Pareto-Front {";
        for (unsigned int i = 0; i < pareto_front.size(); i++) {
            std::cout << (i > 0 ? "," : "");
            std::cout << pareto_front[i];
        }
        std::cout << "}" << std::endl;
    }

    void save_results(GA_Type &ga_obj, const std::string &output_location) {
//        std::ostringstream oss;
        std::vector<unsigned int> paretofront_indices = ga_obj.last_generation.fronts[0];
        for (unsigned int i: paretofront_indices) {
            /* set flow offset and route index */
            auto &X = ga_obj.last_generation.chromosomes[i];
            for (auto &flow: flows) {
                uint32_t flow_id = flow.get().getId() - 1;
                flow.get().setOffset(X.genes.offsets[flow_id]);
                flow.get().setSelectedRouteInx(X.genes.selected_route_idx[flow_id]);
//                spdlog::get("console")->info("flow_{}, route: {}", flow_id, flow.get().getRoutes()[X.genes.selected_route_idx[flow_id]].toString());
            }
            saveGCL(X.genes.offsets, X.genes.selected_route_idx);

            std::string route_file = output_location;
            route_file.append("/" + std::to_string(i) + "SmallRouting.xml");
            saveRoute(route_file);

            std::string gcl_file = output_location;
            gcl_file.append("/" + std::to_string(i) );
            saveGCL(gcl_file, X.genes.selected_route_idx);

            std::string route_file_name = std::to_string(i) + "SmallRouting.xml";
            std::string gcl_file_name = std::to_string(i) + "SmallGCL.xml";
            std::string ini_file = output_location;
            ini_file.append("/" + std::to_string(i) + "SmallTopology.ini");
            savIni(route_file_name, gcl_file_name, ini_file, i);

        }
    }

    void saveRoute(const std::string &output_location) {
        spdlog::get("console")->info("Saving route: {}", output_location);
        std::ofstream output;
        output.open(output_location);
        std::map<uint32_t, std::vector<uint32_t>> link_flows;
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[flow.get().getSelectedRouteInx()].getLinks()) {
                uint32_t link_id = link.get().getId();
                link_flows[link_id].emplace_back(flow.get().getId());
            }
        }
        std::map<node_idx, std::vector<uint32_t >> sw_links;
        for (auto const &sw: swList) {
            for (auto const &[link_id, flowids]: link_flows) {
                if (links[link_id].get().getSrcNode() == sw) {
                    sw_links[Node::nodeToIdx(nodeMap, sw)].emplace_back(link_id);
                }
            }
        }
        output << R"(<?xml version="1.0" encoding="UTF-8" standalone="no"?>)" << std::endl;
        output << R"(<filteringDatabases>)" << std::endl;
        /* iterate all switchs */
        for (auto const &[node_id, links_id]: sw_links) {
            output << "\t" << R"(<filteringDatabase id=")" << nodes[node_id]->getName() << R"(">)" << std::endl;
            output << "\t\t" << R"(<static>)" << std::endl;
            output << "\t\t\t" << R"(<forward>)" << std::endl;
            /* iterate a switch's ports */
            size_t ports = ((Switch *) (nodes[node_id]))->getPortNum();
            for (int i = 0; i < ports; ++i) {
                for (auto &link_id: links_id) {
                    if (uuid_compare(links[link_id].get().getSrcPort().getId(),
                                     ((Switch *) (nodes[node_id]))->getPorts()[i].getId()) == 0) {
                        for (auto const &flow_id: link_flows[link_id]) {
                            output << "\t\t\t\t"
                                   << R"(<multicastAddress macAddress="255-0-00-00-00-)" << std::setw(2) << std::setfill('0') << std::to_string(flow_id)
                                   << R"(" ports=")" << std::to_string(i) << R"("/>)" << std::endl;
                        }
                    }
                }
            }
            output << "\t\t\t" << R"(</forward>)" << std::endl;
            output << "\t\t" << R"(</static>)" << std::endl;
            output << "\t" << R"(</filteringDatabase>)" << std::endl;
        }
        output << R"(</filteringDatabases>)" << std::endl;
        output.close();
    }

    void saveGCL(const std::string &output_location, const std::vector<uint64_t> &selected_route_idx) {
        spdlog::get("console")->info("Saving gcl: {}", output_location);
        std::ofstream output;
        std::string switch_gcl_file = output_location + "SmallGCL.xml";
        output.open(switch_gcl_file);
        std::map<uint32_t, uint64_t> link_hyperperiod = getLinkHyperperiods(flows, selected_route_idx);
        output << R"(<?xml version="1.0" encoding="UTF-8" standalone="no"?>)" << std::endl;
        output << R"(<schedules>)" << std::endl;
        std::vector<std::reference_wrapper<Flow>> wrapper_flows(flows.begin(), flows.end());
        std::ostringstream oss;
        uint32_t hyperPeriod = Util::getHyperPeriod(wrapper_flows, oss);
        output << "\t" << R"(<defaultcycle>)" << std::to_string(hyperPeriod) << R"(ns</defaultcycle>)" << std::endl;

        for (auto &sw: swList) {
            output << "\t" << R"(<switch name=")" << sw->getName() << R"(">)" << std::endl;
            size_t ports = ((Switch *) sw)->getPortNum();
            for (int i = 0; i < ports; ++i) {
                for (auto const &link: links) {
                    if (link.get().getSrcNode() == sw) {
                        if (uuid_compare(link.get().getSrcPort().getId(),
                                         ((Switch *) sw)->getPorts()[i].getId()) == 0 && !link.get().getSrcPort().getGateControlList().empty()) {
                            uint32_t link_id = link.get().getId();
                            output << "\t\t" << R"(<port id=")" << std::to_string(i) << R"(">)" << std::endl;
                            output << "\t\t\t" << R"(<schedule cycleTime=")" << std::to_string(link_hyperperiod[link_id]) << R"(ns">)" << std::endl;
                            uint32_t cur = 0;
                            for (auto const &gcl_entity: links[link_id].get().getSrcPort().getGateControlList()) {
                                uint32_t gap = gcl_entity.getStartTime() - cur;
                                GateControlEntry gateControlEntry;
                                output << "\t\t\t\t" << R"(<entry>)" << std::endl;
                                output << "\t\t\t\t\t" << R"(<length>)" << std::to_string(gap) << R"(ns</length>)" << std::endl;
                                output << "\t\t\t\t\t" << R"(<bitvector>)" << gateControlEntry.toBitVec() << R"(</bitvector>)" << std::endl;
                                output << "\t\t\t\t" << R"(</entry>)" << std::endl;

                                output << "\t\t\t\t" << R"(<entry>)" << std::endl;
                                output << "\t\t\t\t\t" << R"(<length>)" << std::to_string(gcl_entity.getTimeIntervalValue()) << R"(ns</length>)" << std::endl;
                                output << "\t\t\t\t\t" << R"(<bitvector>)" << gcl_entity.toBitVec() << R"(</bitvector>)" << std::endl;
                                output << "\t\t\t\t" << R"(</entry>)" << std::endl;

                                cur = gcl_entity.getStartTime() + gcl_entity.getTimeIntervalValue();
                            }
                            if (cur < link_hyperperiod[link_id]) {
                                uint32_t gap = link_hyperperiod[link_id] - cur;
                                GateControlEntry gateControlEntry;
                                output << "\t\t\t\t" << R"(<entry>)" << std::endl;
                                output << "\t\t\t\t\t" << R"(<length>)" << std::to_string(gap) << R"(ns</length>)" << std::endl;
                                output << "\t\t\t\t\t" << R"(<bitvector>)" << gateControlEntry.toBitVec() << R"(</bitvector>)" << std::endl;
                                output << "\t\t\t\t" << R"(</entry>)" << std::endl;
                            }
                            output << "\t\t\t" << R"(</schedule>)" << std::endl;
                            output << "\t\t" << R"(</port>)" << std::endl;
                        }
                    }
                }
            }
            output << "\t" << R"(</switch>)" << std::endl;
        }
        output << R"(</schedules>)" << std::endl;
        output.close();

        std::map<uint32_t, std::vector<uint32_t>> link_flows;
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[flow.get().getSelectedRouteInx()].getLinks()) {
                uint32_t link_id = link.get().getId();
                link_flows[link_id].emplace_back(flow.get().getId());
            }
        }
        for (auto &es: esList) {
            for (auto const &[link_id, flowids]: link_flows) {
                if (links[link_id].get().getSrcNode() == es) {
                    std::string es_traffic_gen = output_location + es->getName() + ".xml";
                    output.open(es_traffic_gen);
                    output << R"(<?xml version="1.0" encoding="UTF-8" standalone="no"?>)" << std::endl;
                    output << R"(<schedules>)" << std::endl;
                    output << "\t" << R"(<defaultcycle>)" << std::to_string(link_hyperperiod[link_id]) << R"(ns</defaultcycle>)" << std::endl;
                    output << "\t" << R"(<host name=")" << es->getName() << R"(">)" << std::endl;
                    output << "\t\t" << R"(<cycle>)" << std::to_string(link_hyperperiod[link_id]) << R"(ns</cycle>)"
                           << std::endl;
                    for (auto const &flow_id: flowids) {
                        output << "\t\t" << R"(<entry>)" << std::endl;
                        output << "\t\t\t" << R"(<start>)" << std::to_string(flows[flow_id - 1].get().getOffset()) << R"(ns</start>)" << std::endl;
                        output << "\t\t\t" << R"(<queue>)" << flows[flow_id - 1].get().getPriorityCodePoint() << R"(</queue>)" << std::endl;
                        output << "\t\t\t" << R"(<dest>255:0:00:00:00:)" << std::setw(2) << std::setfill('0') << std::to_string(flow_id) << R"(</dest>)" << std::endl;
                        output << "\t\t\t" << R"(<size>)" << std::to_string(flows[flow_id - 1].get().getFrameLength()) << R"(B</size>)" << std::endl;
                        output << "\t\t\t" << R"(<flowId>)" << std::to_string(flow_id) << R"(</flowId>)" << std::endl;
                        output << "\t\t" << R"(</entry>)" << std::endl;
                    }
                    output << "\t" << R"(</host>)" << std::endl;
                    output << R"(</schedules>)" << std::endl;
                    output.close();
                }
            }
        }
    }

    void saveGCL(const std::vector<uint64_t> &offsets,
                 const std::vector<uint64_t> &selected_route_idx) {

        //       link_id   src_port_hyperperiod
        std::map<uint32_t, uint64_t> link_hyperperiod = getLinkHyperperiods(flows, selected_route_idx);
        for (auto &flow: flows) {
            /* Calculate GCL */
            uint32_t flow_id = flow.get().getId() - 1;
            uint32_t offset = offsets[flow_id];
            uint32_t accumulatedDelay = offset;
            uint32_t prop_delay = 0, trans_delay = 0, proc_delay = 0;
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow_id]].getLinks()) {
                link.get().clearGateControlEntry();
                uint32_t link_id = link.get().getId();
                proc_delay = link.get().getSrcNode()->getDpr();
                trans_delay = flow.get().getFrameLength() * link.get().getSrcPort().getMacrotick();
                accumulatedDelay += proc_delay;
                uint32_t sendTimes = link_hyperperiod[link_id] / flow.get().getPeriod();
                for (uint32_t i = 0; i < sendTimes; ++i) {
                    GateControlEntry gateControlEntry;
                    uint32_t start_time = accumulatedDelay + i * flow.get().getPeriod();
                    gateControlEntry.setStartTime(start_time);
                    gateControlEntry.setTimeIntervalValue(trans_delay);
                    for (int j = 0; j < 8; ++j) {
                        if (flow.get().getPriorityCodePoint() == j) {
                            gateControlEntry.setGateStatesValue(j, GATE_OPEN);
                        } else {
                            gateControlEntry.setGateStatesValue(j, GATE_CLOSE);
                        }
                    }
                    link.get().addGateControlEntry(gateControlEntry);
                }
                link.get().sortGCL();
                accumulatedDelay += trans_delay;
                prop_delay = link.get().getLen() * link.get().getPropSpeed();
                accumulatedDelay += prop_delay;
            }
        }
    }

    void savIni(const std::string &route_file, const std::string &gcl_file, const std::string &ini_file, int solution_id) {
        std::ofstream output(ini_file);
        output << "[General]\n"
                  "network = Small\n"
                  "\n"
                  "record-eventlog = false \n"
                  "debug-on-errors = true\n"
                  "result-dir = results_gating\n"
                  "sim-time-limit = 1ms\n"
                  "\n"
                  "# debug\n"
                  "**.displayAddresses = true\n"
                  "**.verbose = true" << std::endl;
        output << "# MAC Addresses" << std::endl;
        for (auto const &es: esList) {
            node_idx id = Node::nodeToIdx(nodeMap, es) + 1;
            output << "**." << es->getName()
                   << R"(.eth.address = "00-00-00-00-00-)" << std::setw(2) << std::setfill('0') << std::hex << id
                   << "\"" << std::endl;
        }
        output << R"(**.frequency = 1THz)" << std::endl;
        output << "# Switches" << std::endl;

        output << R"(**.switch*.processingDelay.delay = 30000ns)" << std::endl;

        output << R"(**.filteringDatabase.database = xmldoc("xml/)" << route_file << R"(", "/filteringDatabases/"))" << std::endl;
        for (auto const &sw: swList) {
            size_t ports = ((Switch *) sw)->getPortNum();
            for (int i = 0; i < ports; ++i) {
                if (isPortInUse(((Switch *) sw)->getPorts()[i])) {
                    output << "**." << sw->getName() << ".eth[" << std::to_string(i) << R"(].queue.gateController.initialSchedule = xmldoc("xml/)" << gcl_file;
                    output << R"(", "/schedules/switch[@name=')" << sw->getName() << R"(']/port[@id=')" << std::to_string(i) << R"(']/schedule"))" << std::endl;
                }
            }
        }
        output << R"(**.gateController.enableHoldAndRelease = true)" << std::endl;
        for (int i = 0; i < 8; ++i) {
            output << R"(**.switch*.eth[*].queuing.tsAlgorithms[)" << std::to_string(i) << R"(].typename = "StrictPriority")" << std::endl;
        }
//        output << R"(**.queues[*].bufferCapacity = 363360b)" << std::endl;
//        output << R"(**.sw*.eth[*].mac.enablePreemptingFrames = false)" << std::endl;

        for (auto &es: esList) {
            if (isPortInUse(((EndSystem*)es)->getPort())) {
                output << "**." << es->getName() << R"(.trafGenSchedApp.initialSchedule = xmldoc("xml/)"
                << std::to_string(solution_id) << es->getName() << R"(.xml"))" << std::endl;
            }
        }

    }

    bool isPortInUse(const Port &port) {
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[flow.get().getSelectedRouteInx()].getLinks()) {
                if (uuid_compare(link.get().getSrcPort().getId(), port.getId()) == 0)
                    return true;
            }
        }
        return false;
    }


};


#endif //SCHEDPLUS_MYFUNCTIONS_H
