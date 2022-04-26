//
// Created by faniche on 2022/3/21.
//

#ifndef SCHEDPLUS_GA_H
#define SCHEDPLUS_GA_H


#include <vector>
#include <fstream>
#include "../components/Flow.h"
#include "../../lib/openGA.hpp"
#include "GA_Solution.h"
#include "MyFunctions.h"

static bool compareFlowWithPCP(const Flow &flow1, const Flow &flow2) {
    return flow1.getPriorityCodePoint() > flow2.getPriorityCodePoint();
}

static bool compareFlowWithPeriod(const Flow &flow1, const Flow &flow2) {
    return flow1.getPeriod() < flow2.getPeriod();
}

static void openGACal() {
    std::ostringstream oss;
    /* End systems */
    Node *es0 = createNode(END_SYSTEM, "es0", 30000);
    Node *es1 = createNode(END_SYSTEM, "es1", 30000);
    Node *es2 = createNode(END_SYSTEM, "es2", 30000);
    Node *es3 = createNode(END_SYSTEM, "es3", 30000);
    Node *es4 = createNode(END_SYSTEM, "es4", 30000);
    Node *es5 = createNode(END_SYSTEM, "es5", 30000);
    Node *es6 = createNode(END_SYSTEM, "es6", 30000);
    Node *es7 = createNode(END_SYSTEM, "es7", 30000);
    /* Switches */
    Node *sw0 = createNode(SWITCH, "sw0", 30000);
    Node *sw1 = createNode(SWITCH, "sw1", 30000);
    Node *sw2 = createNode(SWITCH, "sw2", 30000);
    Node *sw3 = createNode(SWITCH, "sw3", 30000);
    Node *sw4 = createNode(SWITCH, "sw4", 30000);
    std::vector<Node *> nodes{es0, es1, es2, es3, es4, es5, es6, es7, sw0, sw1, sw2, sw3, sw4};
    std::vector<Node *> esList{es0, es1, es2, es3, es4, es5, es6, es7};
    std::vector<Node *> swList{sw0, sw1, sw2, sw3, sw4};
    std::map<node_idx , Node *> nodeMap;
    for (node_idx i = 0; i < nodes.size(); ++i) {
        nodeMap[i] = nodes[i];
    }

    /* Links connected end systems to switches */
    FullDuplexLink link_00(es0, sw0, ((EndSystem *) es0)->getPort(), ((Switch *) sw0)->getPorts()[0]);
    FullDuplexLink link_01(es1, sw0, ((EndSystem *) es1)->getPort(), ((Switch *) sw0)->getPorts()[1]);
    FullDuplexLink link_02(es2, sw1, ((EndSystem *) es2)->getPort(), ((Switch *) sw1)->getPorts()[0]);
    FullDuplexLink link_03(es3, sw1, ((EndSystem *) es3)->getPort(), ((Switch *) sw1)->getPorts()[1]);
    FullDuplexLink link_04(es4, sw2, ((EndSystem *) es4)->getPort(), ((Switch *) sw2)->getPorts()[0]);
    FullDuplexLink link_05(es5, sw2, ((EndSystem *) es5)->getPort(), ((Switch *) sw2)->getPorts()[1]);
    FullDuplexLink link_06(es6, sw3, ((EndSystem *) es6)->getPort(), ((Switch *) sw3)->getPorts()[0]);
    FullDuplexLink link_07(es7, sw3, ((EndSystem *) es7)->getPort(), ((Switch *) sw3)->getPorts()[1]);
    /* Links connected four switches */
    FullDuplexLink link_08(sw0, sw1, ((Switch *) sw0)->getPorts()[2], ((Switch *) sw1)->getPorts()[2]);
    FullDuplexLink link_09(sw0, sw2, ((Switch *) sw0)->getPorts()[3], ((Switch *) sw2)->getPorts()[2]);
    FullDuplexLink link_10(sw2, sw3, ((Switch *) sw2)->getPorts()[3], ((Switch *) sw3)->getPorts()[2]);
    FullDuplexLink link_11(sw1, sw3, ((Switch *) sw1)->getPorts()[3], ((Switch *) sw3)->getPorts()[3]);

    FullDuplexLink link_12(sw0, sw4, ((Switch *) sw0)->getPorts()[4], ((Switch *) sw4)->getPorts()[0]);
    FullDuplexLink link_13(sw1, sw4, ((Switch *) sw1)->getPorts()[4], ((Switch *) sw4)->getPorts()[1]);
    FullDuplexLink link_14(sw2, sw4, ((Switch *) sw2)->getPorts()[4], ((Switch *) sw4)->getPorts()[2]);
    FullDuplexLink link_15(sw3, sw4, ((Switch *) sw3)->getPorts()[4], ((Switch *) sw4)->getPorts()[3]);

    /* Add links to vectors */
    std::vector<DirectedLink> links{link_00.getLinks()[0], link_00.getLinks()[1],
                                    link_01.getLinks()[0], link_01.getLinks()[1],
                                    link_02.getLinks()[0], link_02.getLinks()[1],
                                    link_03.getLinks()[0], link_03.getLinks()[1],
                                    link_04.getLinks()[0], link_04.getLinks()[1],
                                    link_05.getLinks()[0], link_05.getLinks()[1],
                                    link_06.getLinks()[0], link_06.getLinks()[1],
                                    link_07.getLinks()[0], link_07.getLinks()[1],
                                    link_08.getLinks()[0], link_08.getLinks()[1],
                                    link_09.getLinks()[0], link_09.getLinks()[1],
                                    link_10.getLinks()[0], link_10.getLinks()[1],
                                    link_11.getLinks()[0], link_11.getLinks()[1],
                                    link_12.getLinks()[0], link_12.getLinks()[1],
                                    link_13.getLinks()[0], link_13.getLinks()[1],
                                    link_14.getLinks()[0], link_14.getLinks()[1],
                                    link_15.getLinks()[0], link_15.getLinks()[1]};
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
    for (int i = 0; i < 100; ++i) {
        node_idx a, b;
        do {
            a = util.getRandESIdx(esList);
            b = util.getRandESIdx(esList);
        } while (a == b);
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
        flow.setFrameLength(Flow::getRandomFrameLength(pcp));
        util.calAllRoutes(nodeMap, flow, graph, links);
        flows.push_back(flow);
        oss.str("");
        flow.toString(oss);
        spdlog::info("flow_{}: {}", i, oss.str());
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
    for (auto &[src, _flows]: flowGroup) {
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

//    std::vector<std::reference_wrapper<Flow>> wrapper_flows(flows.begin(), flows.end());
    uint32_t hyperPeriod = Util::getHyperPeriod(flows, oss);

    MyFunctions myobject("/home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/result.txt",
                         nodes, esList, swList, nodeMap, links, flows, flowGroup);

    EA::Chronometer timer;
    timer.tic();

    GA_Type ga_obj;
    ga_obj.problem_mode = EA::GA_MODE::NSGA_III;
    ga_obj.multi_threading = true;
//    ga_obj.idle_delay_us = 1; // switch between threads quickly
    ga_obj.dynamic_threading = true;
    ga_obj.verbose = false;
    ga_obj.population = 100;
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

    myobject.save_results(ga_obj);

}


#endif //SCHEDPLUS_GA_H
