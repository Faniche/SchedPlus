//
// Created by faniche on 22-5-19.
//

#ifndef SCHEDPLUS_GA_LINE_2_2_H
#define SCHEDPLUS_GA_LINE_2_2_H

#include <vector>
#include <fstream>
#include "../components/Flow.h"
#include "../../lib/openGA.hpp"
#include "GA_Solution.h"
#include "MyFunctions.h"

class GA_line_2_1 {
public:
    static void openGACal() {
        std::ostringstream oss;
        /* End systems */
        Node *es00 = createNode(END_SYSTEM, "es00", 0);
        Node *es01 = createNode(END_SYSTEM, "es01", 0);

        /* Switches */
        Node *sw0 = createNode(SWITCH, "switch0", 30000);
        std::vector<Node *> nodes{es00, es01, sw0};
        std::vector<Node *> esList{es00, es01};
        std::vector<Node *> swList{sw0};
        std::map<node_idx , Node *> nodeMap;
        for (node_idx i = 0; i < nodes.size(); ++i) {
            nodeMap[i] = nodes[i];
        }

        /* Links connected end systems to switches */
        FullDuplexLink link_00(es00, sw0, ((EndSystem *) es00)->getPort(), ((Switch *) sw0)->getPorts()[0]);
        FullDuplexLink link_01(es01, sw0, ((EndSystem *) es01)->getPort(), ((Switch *) sw0)->getPorts()[1]);

        /* Add links to vectors */
        std::vector<DirectedLink> links{link_00.getLinks()[0], link_00.getLinks()[1],
                                        link_01.getLinks()[0], link_01.getLinks()[1]};
        for (uint32_t i = 0; i < links.size(); ++i) {
            links[i].setId(i);
        }
//    std::vector<std::reference_wrapper<DirectedLink>> wrapper_links(links.begin(), links.end());
        /* Adjacency matrix */
        /* Init a graph of int value to calculate routes later*/
        Graph graph(nodes.size());
        Graph::initGraph(nodeMap, links, graph);

        Util util;
        /* The set of flows */
        std::vector<Flow> flows;
        size_t flow_num = 2;
        for (int i = 1; i <= flow_num; ++i) {
            node_idx a, b;
            if (i <= flow_num / 2) {
                a = 0;
                b = 1;
            } else {
                a = 1;
                b = 0;
            }
            PRIORITY_CODE_POINT pcp = util.getRandPCP();
            Flow flow(i,
                      Flow::getRandomPeriod(pcp),
                      pcp,
                      nodeMap.at(a),
                      nodeMap.at(b),
                      false,
                      1,
                      false);
            if (pcp == P6) {
                /* unit: ns*/
                DeliveryGuarantee deliveryGuarantee(DDL, flow.getPeriod());
                flow.addDeliveryGuarantee(deliveryGuarantee);
            } else if (pcp == P5) {
                /* Typically less than 90% of period. */
                /* unit: ns*/
                DeliveryGuarantee deliveryGuarantee(E2E, flow.getPeriod() / 10);
                flow.addDeliveryGuarantee(deliveryGuarantee);
            }
//            flow.setFrameLength(Flow::getRandomFrameLength(pcp) + 22);
            flow.setFrameLength(Flow::getRandomFrameLength(pcp) + 22);
            util.calAllRoutes(nodeMap, flow, graph, links);
            flows.push_back(flow);
            oss.str("");
            flow.toString(oss);
            spdlog::get("console")->info("flow_{}: {}", i, oss.str());
        }
        std::map<node_idx, std::vector<std::reference_wrapper<Flow>>> flowGroup;
        /* Group the flow with src */
        for (auto &flow: flows) {
            node_idx key = Node::nodeToIdx(nodeMap, flow.getSrc());
            flowGroup[key].emplace_back(flow);
        }

        /* Sort the flow with PCP */
        spdlog::set_level(spdlog::level::info);
        for (auto &[src, _flows]: flowGroup) {
            std::sort(_flows.begin(), _flows.end(), Util::compareFlowWithPCP);
        }
        spdlog::get("console")->debug("Group the flow with pcp:{}", oss.str());
        /* Sort the flow with period on every source node */
        for (auto &[src, _flows]: flowGroup) {
            auto start = _flows.begin();
            auto end = _flows.begin();
            for (; end != _flows.end().operator-(1); ++end) {
                if (end->get().getPriorityCodePoint() != (end.operator+(1))->get().getPriorityCodePoint()) {
                    std::sort(start, end.operator+(1), Util::compareFlowWithPeriod);
                    start = end.operator+(1);
                }
            }
            std::sort(start, ++end, Util::compareFlowWithPeriod);
        }

        MyFunctions myobject("/home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/result.txt",
                             nodes, esList, swList, nodeMap, links, flows, flowGroup);

        EA::Chronometer timer;
        timer.tic();

        GA_Type ga_obj;
        ga_obj.problem_mode = EA::GA_MODE::NSGA_III;
        ga_obj.multi_threading = true;
//    ga_obj.idle_delay_us = 1; // switch between threads quickly
        ga_obj.dynamic_threading = true;
        ga_obj.verbose = true;
        ga_obj.population = 20;
        ga_obj.generation_max = 100;

        using std::bind;
        using std::placeholders::_1;
        using std::placeholders::_2;
        using std::placeholders::_3;

        ga_obj.calculate_MO_objectives = bind(&MyFunctions::calculate_MO_objectives, &myobject, _1);
        ga_obj.init_genes =  bind(&MyFunctions::init_genes, &myobject, _1, _2);
        ga_obj.eval_solution = bind(&MyFunctions::eval_solution, &myobject, _1, _2);
        ga_obj.mutate = bind(&MyFunctions::mutate, &myobject, _1, _2, _3);
        ga_obj.crossover = bind(&MyFunctions::crossover, &myobject, _1, _2, _3);
        ga_obj.MO_report_generation = bind(&MyFunctions::MO_report_generation, &myobject, _1, _2, _3);

        ga_obj.crossover_fraction = 0.9;
        ga_obj.mutation_rate = 0.2;
        ga_obj.solve();

        std::cout << "The problem is optimized in " << timer.toc() << " seconds." << std::endl;
        myobject.save_results(ga_obj, "/home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/xml/small");

    }
};


#endif //SCHEDPLUS_GA_LINE_2_2_H
