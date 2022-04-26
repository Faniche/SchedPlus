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

    static bool compIntervals(const std::pair<uint32_t, uint32_t> &p1, const std::pair<uint32_t, uint32_t> &p2) {
        return p1.first < p2.first;
    }

    static void setTimeIntervals(const std::vector<std::reference_wrapper<Flow>> &flows, TTFlows &p) {
        spdlog::set_level(spdlog::level::info);
        spdlog::set_pattern("%Y-%m-%d %H:%M:%S %t %! %l %@ : %v");
        std::map<uint32_t, uint32_t> mem_allocate;

        for (auto const &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            for (auto &link: flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks()) {
                uint32_t sendTimes = flow.get().getHyperperiod() / flow.get().getPeriod();
                for (int i = 0; i < sendTimes; ++i) {
                    if (mem_allocate.contains(link.get().getId())) {
                        mem_allocate[link.get().getId()] += sendTimes;
                    } else {
                        mem_allocate[link.get().getId()] = sendTimes;
                    }
                }
            }
        }

        for (auto &[link_id, gcl_entities]: p.transmit_intervals) {
            gcl_entities.reserve(mem_allocate[link_id]);
        }


        for (auto const &flow: flows) {
            /* Calculate GCL */
            uint32_t flow_id = flow.get().getId();
            uint32_t offset = p.offsets[flow_id];
            uint32_t accumulatedDelay = offset;
            uint32_t prop_delay = 0, trans_delay = 0, proc_delay = 0;
            for (auto &link: flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks()) {
                proc_delay = link.get().getSrcNode()->getDpr();
                trans_delay = flow.get().getFrameLength() * link.get().getSrcPort().getMacrotick();
                accumulatedDelay += proc_delay;
                uint32_t sendTimes = flow.get().getHyperperiod() / flow.get().getPeriod();
//                uint32_t low_bound = p.transmit_intervals[link.get().getId()].size();
                for (uint32_t i = 0; i < sendTimes; ++i) {
                    uint32_t start_time = accumulatedDelay + i * flow.get().getPeriod();

                    p.transmit_intervals[link.get().getId()].emplace_back(std::pair(start_time, trans_delay));
                }
                accumulatedDelay += trans_delay;
                prop_delay = link.get().getLen() * link.get().getPropSpeed();
                accumulatedDelay += prop_delay;
                spdlog::debug("flow_id: {}, link_id: {}, interval no: {}.", flow_id, link.get().getId(), sendTimes);
            }
        }
        for (auto &[link_idx, intervals]: p.transmit_intervals) {
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
        output_file.open(output_location);
        links.assign(_links.begin(), _links.end());
        flows.assign(_flows.begin(), _flows.end());
        output_file << "step" << "\t" << "x_best" << "\t" << "y_best" << "\t" << "cost_avg" << "\t" << "cost_best"
                    << "\n";
    }

    ~MyFunctions() {
        output_file.close();
    }

    void init_genes(TTFlows &p, const std::function<double(void)> &rnd01) {
        p.offsets.assign(flows.size(), 0);
        p.selected_route_idx.assign(flows.size(), 0);

        /* Initialize the offset and route index of flow. */
        for (auto const &flow: flows) {
            /* Transmit delay of frame, unit: ns */
            //      delay     =  frame_length   *    port_macrotick
            int srcTransDelay =
                    flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
            uint32_t offset = (flow.get().getPeriod() - srcTransDelay) * rnd01();
            p.offsets[flow.get().getId()] = offset;
            uint32_t route_idx = (flow.get().getRoutes().size()) * rnd01();
            p.selected_route_idx[flow.get().getId()] = route_idx;
        }
        setTimeIntervals(flows, p);
    }


    bool eval_solution(const TTFlows &p, MyMiddleCost &c) {
        c.delivery_guarantees.assign(flows.size(), 0);
        bool gcl_collision = true;
        bool ddl_met = true;
        bool e2e_met = true;
        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            /* Check delivery guarantees */
            int e2e = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getE2E();
            if (flow.get().getPriorityCodePoint() == P6) {
                c.delivery_guarantees[flow_id] = p.offsets[flow_id] + e2e;
                if (c.delivery_guarantees[flow_id] > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                    ddl_met = false;
                if (c.delivery_guarantees[flow_id] > c.ddl)
                    c.ddl = c.delivery_guarantees[flow_id];
            } else if (flow.get().getPriorityCodePoint() == P6) {
                if (e2e > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                    ddl_met = false;
                if (e2e > c.e2e)
                    c.e2e = e2e;
            }

            for (auto &[link_idx, intervals]: p.transmit_intervals) {
                for (int i = 0; i < intervals.size() - 1; ++i)
                    if (intervals[i].second > intervals[i + 1].first) {
                        gcl_collision = false;
                        break;
                    }
                if (!gcl_collision) break;
            }
        }
        bool ret = gcl_collision && ddl_met && e2e_met;
        spdlog::set_level(spdlog::level::info);
        if (ret) {
            spdlog::debug("***********Correct solution***********");
            std::for_each(flows.begin(), flows.end(),
                          [&](std::reference_wrapper<Flow> flow) {
                              spdlog::debug("flow_id: {}, offset: {}", flow.get().getId(),
                                            p.offsets[flow.get().getId()]);
                          });
            spdlog::debug("**************************************");
        } else {
            spdlog::debug("*************Bad solution*************");
        }
        return ret; // solution is accepted
    }

    TTFlows mutate(
            const TTFlows &X_base,
            const std::function<double(void)> &rnd01,
            double shrink_scale) {
        TTFlows X_new;
        const double mu = 0.2 * shrink_scale; // mutation radius (adjustable)
        X_new = X_base;
        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            int offset = 0, route = 0;
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
            int offset = r_offset * X1.offsets[i] + (1.0 - r_offset) * X2.offsets[i];
            X_new.offsets[i] = offset;
            int route = r_route * X1.selected_route_idx[i] + (1.0 - r_route) * X2.selected_route_idx[i];
            X_new.selected_route_idx[i] = route;
        }
        return X_new;
    }

    std::vector<double> calculate_MO_objectives(const GA_Type::thisChromosomeType &X) {
        return {
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

    void save_results(const GA_Type &ga_obj) {
        std::ostringstream oss;
        std::vector<unsigned int> paretofront_indices = ga_obj.last_generation.fronts[0];
        for (unsigned int i: paretofront_indices) {
            output_file << "solution_" << i << std::endl;
            output_file << std::setw(10) << std::left << "flow_id" << "\t"
                        << std::setw(10) << std::left << "offset" << "\t"
                        << std::setw(10) << std::left << "route_index" << std::endl;
            const auto &X = ga_obj.last_generation.chromosomes[i];

            std::for_each(flows.begin(), flows.end(),
                          [&](std::reference_wrapper<Flow> flow) {
                              output_file << std::setw(10) << std::left << flow.get().getId() << "\t"
                                          << std::setw(10) << std::left << X.genes.offsets[flow.get().getId()] << "\t"
                                          << std::setw(10) << std::left << X.genes.selected_route_idx[flow.get().getId()]
                                          << std::endl;
                          });
            for (auto &[link_id, gcl_entities]: X.genes.transmit_intervals) {
                output_file << "link_idx " << link_id << std::endl;
                output_file << std::setw(10) << std::left << "start" << "\t"
                            << std::setw(10) << std::left << "interval" << std::endl;
                std::for_each(gcl_entities.begin(), gcl_entities.end(),
                              [&](const std::pair<uint32_t, uint32_t> &gcl_entity) {
                                  output_file << std::setw(10) << std::left << gcl_entity.first << "\t"
                                              << std::setw(10) << std::left << gcl_entity.second << std::endl;
                              });
            }

            output_file
                    << i << "\t"
                    << X.middle_costs.e2e << "\t"
                    << X.middle_costs.ddl << std::endl;

        }
        output_file.close();
    }
};


#endif //SCHEDPLUS_MYFUNCTIONS_H
