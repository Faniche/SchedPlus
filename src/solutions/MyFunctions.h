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
    std::mutex gcl_lock;

    static bool compareFlowWithPCP(const Flow &flow1, const Flow &flow2) {
        return flow1.getPriorityCodePoint() > flow2.getPriorityCodePoint();
    }

    static bool compareFlowWithPeriod(const Flow &flow1, const Flow &flow2) {
        return flow1.getPeriod() < flow2.getPeriod();
    }

public:
    std::ofstream output_file;

    MyFunctions(const std::string &output_location,
                std::vector<Node *> nodes,
                std::vector<Node *> esList,
                std::vector<Node *> swList,
                std::map<node_idx, Node *> nodeMap,
//                std::vector<std::reference_wrapper<DirectedLink>> _links,
                std::vector<DirectedLink> &_links,
                std::vector<Flow> &_flows) : nodes(std::move(nodes)),
                                             esList(std::move(esList)),
                                             swList(std::move(swList)),
                                             nodeMap(std::move(nodeMap)) {
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
        std::ostringstream oss;
        for (auto &flow: flows) {
            p.flows.emplace_back(flow.get());
        }
        /* Group the flow with src */
        for (auto &flow: p.flows) {
            node_idx key = Node::nodeToIdx(nodeMap, flow.get().getSrc());
            p.flowGroup[key].emplace_back(flow.get());
        }
        /* Sort the flow with PCP */
        spdlog::set_level(spdlog::level::info);
        for (auto &[src, _flows]: p.flowGroup) {
//            oss.str("");
//            std::for_each(_flows.begin(), _flows.end(),
//                          [&](std::reference_wrapper<Flow> flow) {
//                              oss << flow.get().getPriorityCodePoint() << " ";
//                          });
            spdlog::debug("Before sorting: {}", oss.str());
//            oss.str("");
            std::sort(_flows.begin(), _flows.end(), compareFlowWithPCP);
//            std::for_each(_flows.begin(), _flows.end(),
//                          [&](std::reference_wrapper<Flow> flow) {
//                              oss << flow.get().getPriorityCodePoint() << " ";
//                          });
            spdlog::debug("After sorting: {}", oss.str());
        }
        spdlog::debug("Group the flow with pcp:{}", oss.str());
        /* Sort the flow with period on every source node */
        for (auto &[src, _flows]: p.flowGroup) {
//            oss.str("");
//            spdlog::debug("src idx: {}", src);
//            std::for_each(_flows.begin(), _flows.end(),
//                          [&](std::reference_wrapper<Flow> flow) {
//                              oss << flow.get().getPeriod() << " ";
//                          });
            spdlog::debug("Before sorting: {}", oss.str());
//            oss.str("");
            auto start = _flows.begin();
            auto end = _flows.begin();
            for (; end != _flows.end().operator-(1); ++end) {
                if (end->get().getPriorityCodePoint() != (end.operator+(1))->get().getPriorityCodePoint()) {
                    std::sort(start, end.operator+(1), compareFlowWithPeriod);
                    start = end.operator+(1);
                }
            }
            std::sort(start, ++end, compareFlowWithPeriod);
//            std::for_each(_flows.begin(), _flows.end(),
//                          [&](std::reference_wrapper<Flow> flow) {
//                              oss << flow.get().getPeriod() << " ";
//                          });
            spdlog::debug("After sorting: {}", oss.str());
        }
        /* Initialize the offset and route index of flow. */
        for (auto &flow: p.flows) {
            /* Transmit delay of frame, unit: ns */
            //      delay     =       frame_length          *                      port_macrotick
            int srcTransDelay =
                    flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
            flow.get().setOffset((flow.get().getPeriod() * 1000 - srcTransDelay) * rnd01());
            flow.get().setSelectedRouteInx((flow.get().getRoutes().size()) * rnd01());
            flow.get().setDeliveryGuarantee();
        }
    }

    bool eval_solution(const TTFlows &p, MyMiddleCost &c) {
        bool gcl_collision = true;
        bool ddl_met = true;
        bool e2e_met = true;

        for (auto &flow: p.flows) {
            /* Calculate GCL */
            if (!flow.get().addGateControlEntry(gcl_lock))
                gcl_collision = false;
            /* Check delivery guarantees */
            for (auto &deliveryGuarantee: flow.get().getDeliveryGuarantees()) {
                if (deliveryGuarantee.getType() == DDL) {
                    if (deliveryGuarantee.getLowerVal() > c.ddl)
                        c.ddl = deliveryGuarantee.getLowerVal();
                    if (deliveryGuarantee.getLoverObj() > deliveryGuarantee.getLowerVal())
                        ddl_met = false;
                } else if (deliveryGuarantee.getType() == E2E) {
                    if (deliveryGuarantee.getLowerVal() > c.e2e)
                        c.e2e = deliveryGuarantee.getLowerVal();
                    if (deliveryGuarantee.getLoverObj() > deliveryGuarantee.getLowerVal())
                        e2e_met = false;
                }
            }
        }
        return gcl_collision && ddl_met && e2e_met; // solution is accepted
    }

    TTFlows mutate(
            const TTFlows &X_base,
            const std::function<double(void)> &rnd01,
            double shrink_scale) {
        TTFlows X_new;
        const double mu = 0.2 * shrink_scale; // mutation radius (adjustable)
        X_new = X_base;
        for (auto &flow: X_new.flows) {
            int offset = 0, route = 0;
            bool in_range;
            do {
                in_range = true;
                offset = flow.get().getOffset() + mu * (rnd01() - rnd01());
                int srcTransDelay =
                        flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
                in_range = in_range && offset >= 0 && offset < flow.get().getPeriod() * 1000 - srcTransDelay;
                route = flow.get().getSelectedRouteInx() + mu * (rnd01() - rnd01());
                in_range = in_range && route >= 0 && route < flow.get().getRoutes().size();
                            } while (!in_range);
            flow.get().setOffset(offset);
            flow.get().setSelectedRouteInx(route);
            flow.get().setDeliveryGuarantee();
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
        for (int i = 0; i < X1.flows.size(); ++i) {
            int offset = r_offset * X1.flows[i].get().getOffset() + (1.0 - r_offset) * X2.flows[i].get().getOffset();
            X_new.flows[i].get().setOffset(offset);
            int route = r_route * X1.flows[i].get().getSelectedRouteInx() +
                        (1.0 - r_route) * X2.flows[i].get().getSelectedRouteInx();
            X_new.flows[i].get().setSelectedRouteInx(route);
            X_new.flows[i].get().setDeliveryGuarantee();
        }
        return X_new;
    }

    std::vector<double> calculate_MO_objectives(const GA_Type::thisChromosomeType &X) {
        return {
                X.middle_costs.delayQueue,
                X.middle_costs.bandwidthUsage
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
        std::ofstream output_file;
        std::ostringstream oss;
        output_file.open("paretofront.txt");
        output_file
                << "N"
                << "\t" << "delayDelay"
                << "\t" << "bandwidthUsage"
                << "\t" << "solution" << "\n";
        std::vector<unsigned int> paretofront_indices = ga_obj.last_generation.fronts[0];
        for (unsigned int i: paretofront_indices) {
            const auto &X = ga_obj.last_generation.chromosomes[i];
            output_file
                    << i << "\t"
                    << X.middle_costs.delayQueue << "\t"
                    << X.middle_costs.bandwidthUsage << "\t"
                    << X.genes.to_string(oss) << "\n";

        }
        output_file.close();
    }
};


#endif //SCHEDPLUS_MYFUNCTIONS_H
