//
// Created by faniche on 2022/3/21.
//

#ifndef SCHEDPLUS_GA_H
#define SCHEDPLUS_GA_H


#include <vector>
#include <fstream>
#include "../components/Flow.h"
#include "../../lib/openGA/openGA.hpp"
#include "GA_Solution.h"
#include "../Utils.h"

void openGACal(const size_t flow_num,
               std::vector<Node *> &nodes,
               std::vector<Node *> &esList,
               std::vector<Node *> &swList,
               std::map<node_idx , Node *> &nodeMap,
               std::vector<DirectedLink> &links,
               std::vector<Flow> &flows) {
    std::ostringstream oss;
    /* End systems */
    Node *es00 = createNode(END_SYSTEM, "es00", 0);
    Node *es01 = createNode(END_SYSTEM, "es01", 0);
    Node *es02 = createNode(END_SYSTEM, "es02", 0);
    Node *es03 = createNode(END_SYSTEM, "es03", 0);
    Node *es04 = createNode(END_SYSTEM, "es04", 0);
    Node *es05 = createNode(END_SYSTEM, "es05", 0);
    Node *es06 = createNode(END_SYSTEM, "es06", 0);
    Node *es07 = createNode(END_SYSTEM, "es07", 0);
    Node *es08 = createNode(END_SYSTEM, "es08", 0);
    Node *es09 = createNode(END_SYSTEM, "es09", 0);
    Node *es10 = createNode(END_SYSTEM, "es10", 0);
    Node *es11 = createNode(END_SYSTEM, "es11", 0);
    Node *es12 = createNode(END_SYSTEM, "es12", 0);
    Node *es13 = createNode(END_SYSTEM, "es13", 0);
    Node *es14 = createNode(END_SYSTEM, "es14", 0);
    Node *es15 = createNode(END_SYSTEM, "es15", 0);
    Node *es16 = createNode(END_SYSTEM, "es16", 0);
    Node *es17 = createNode(END_SYSTEM, "es17", 0);


    /* Switches */
    Node *sw0 = createNode(SWITCH, "switch0", 30000);
    Node *sw1 = createNode(SWITCH, "switch1", 30000);
    Node *sw2 = createNode(SWITCH, "switch2", 30000);
    Node *sw3 = createNode(SWITCH, "switch3", 30000);
    Node *sw4 = createNode(SWITCH, "switch4", 30000);
    Node *sw5 = createNode(SWITCH, "switch5", 30000);
    Node *sw6 = createNode(SWITCH, "switch6", 30000);
    nodes.insert(nodes.end(), {es00, es01, es02, es03, es04, es05,
                              es06, es07, es08, es09, es10, es11,
                              es12, es13, es14, es15, es16, es17,
                              sw0, sw1, sw2, sw3, sw4, sw5, sw6});
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i]->setId(i);
    }
    esList.insert(esList.end(), {es00, es01, es02, es03, es04, es05,
                               es06, es07, es08, es09, es10, es11,
                               es12, es13, es14, es15, es16, es17});
    swList.insert(swList.end(), {sw0, sw1, sw2, sw3, sw4, sw5, sw6});
    for (node_idx i = 0; i < nodes.size(); ++i) {
        nodeMap[i] = nodes[i];
    }

    /* Links connected end systems to switches */
    FullDuplexLink link_00(es00, sw0, ((EndSystem *) es00)->getPort(), ((Switch *) sw0)->getPorts()[0]);
    FullDuplexLink link_01(es01, sw0, ((EndSystem *) es01)->getPort(), ((Switch *) sw0)->getPorts()[1]);
    FullDuplexLink link_02(es02, sw0, ((EndSystem *) es02)->getPort(), ((Switch *) sw0)->getPorts()[2]);

    FullDuplexLink link_03(es03, sw1, ((EndSystem *) es03)->getPort(), ((Switch *) sw1)->getPorts()[0]);
    FullDuplexLink link_04(es04, sw1, ((EndSystem *) es04)->getPort(), ((Switch *) sw1)->getPorts()[1]);
    FullDuplexLink link_05(es05, sw1, ((EndSystem *) es05)->getPort(), ((Switch *) sw1)->getPorts()[2]);

    FullDuplexLink link_06(es06, sw2, ((EndSystem *) es06)->getPort(), ((Switch *) sw2)->getPorts()[0]);
    FullDuplexLink link_07(es07, sw2, ((EndSystem *) es07)->getPort(), ((Switch *) sw2)->getPorts()[1]);
    FullDuplexLink link_08(es08, sw2, ((EndSystem *) es08)->getPort(), ((Switch *) sw2)->getPorts()[2]);

    FullDuplexLink link_09(es09, sw3, ((EndSystem *) es09)->getPort(), ((Switch *) sw3)->getPorts()[0]);
    FullDuplexLink link_10(es10, sw3, ((EndSystem *) es10)->getPort(), ((Switch *) sw3)->getPorts()[1]);
    FullDuplexLink link_11(es11, sw3, ((EndSystem *) es11)->getPort(), ((Switch *) sw3)->getPorts()[2]);

    FullDuplexLink link_12(es12, sw4, ((EndSystem *) es12)->getPort(), ((Switch *) sw4)->getPorts()[0]);
    FullDuplexLink link_13(es13, sw4, ((EndSystem *) es13)->getPort(), ((Switch *) sw4)->getPorts()[1]);
    FullDuplexLink link_14(es14, sw4, ((EndSystem *) es14)->getPort(), ((Switch *) sw4)->getPorts()[2]);

    FullDuplexLink link_15(es15, sw5, ((EndSystem *) es15)->getPort(), ((Switch *) sw5)->getPorts()[0]);
    FullDuplexLink link_16(es16, sw5, ((EndSystem *) es16)->getPort(), ((Switch *) sw5)->getPorts()[1]);
    FullDuplexLink link_17(es17, sw5, ((EndSystem *) es17)->getPort(), ((Switch *) sw5)->getPorts()[2]);

    /* Links connected four switches */
    FullDuplexLink link_18(sw0, sw6, ((Switch *) sw0)->getPorts()[3], ((Switch *) sw6)->getPorts()[0]);
    FullDuplexLink link_19(sw1, sw6, ((Switch *) sw1)->getPorts()[3], ((Switch *) sw6)->getPorts()[1]);
    FullDuplexLink link_20(sw2, sw6, ((Switch *) sw2)->getPorts()[3], ((Switch *) sw6)->getPorts()[2]);
    FullDuplexLink link_21(sw3, sw6, ((Switch *) sw3)->getPorts()[3], ((Switch *) sw6)->getPorts()[3]);
    FullDuplexLink link_22(sw4, sw6, ((Switch *) sw4)->getPorts()[3], ((Switch *) sw6)->getPorts()[4]);
    FullDuplexLink link_23(sw5, sw6, ((Switch *) sw5)->getPorts()[3], ((Switch *) sw6)->getPorts()[5]);

    FullDuplexLink link_24(sw0, sw1, ((Switch *) sw0)->getPorts()[4], ((Switch *) sw1)->getPorts()[5]);
    FullDuplexLink link_25(sw1, sw2, ((Switch *) sw1)->getPorts()[4], ((Switch *) sw2)->getPorts()[5]);
    FullDuplexLink link_26(sw2, sw3, ((Switch *) sw2)->getPorts()[4], ((Switch *) sw3)->getPorts()[5]);
    FullDuplexLink link_27(sw3, sw4, ((Switch *) sw3)->getPorts()[4], ((Switch *) sw4)->getPorts()[5]);
    FullDuplexLink link_28(sw4, sw5, ((Switch *) sw4)->getPorts()[4], ((Switch *) sw5)->getPorts()[5]);
    FullDuplexLink link_29(sw5, sw0, ((Switch *) sw5)->getPorts()[4], ((Switch *) sw0)->getPorts()[5]);

    /* Add links to vectors */
    links.insert(links.end(), {link_00.getLinks()[0], link_00.getLinks()[1],
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
                                    link_15.getLinks()[0], link_15.getLinks()[1],
                                    link_16.getLinks()[0], link_16.getLinks()[1],
                                    link_17.getLinks()[0], link_17.getLinks()[1],
                                    link_18.getLinks()[0], link_18.getLinks()[1],
                                    link_19.getLinks()[0], link_19.getLinks()[1],
                                    link_20.getLinks()[0], link_20.getLinks()[1],
                                    link_21.getLinks()[0], link_21.getLinks()[1],
                                    link_22.getLinks()[0], link_22.getLinks()[1],
                                    link_23.getLinks()[0], link_23.getLinks()[1],
                                    link_24.getLinks()[0], link_24.getLinks()[1],
                                    link_25.getLinks()[0], link_25.getLinks()[1],
                                    link_26.getLinks()[0], link_26.getLinks()[1],
                                    link_27.getLinks()[0], link_27.getLinks()[1],
                                    link_28.getLinks()[0], link_28.getLinks()[1],
                                    link_29.getLinks()[0], link_29.getLinks()[1]});
    for (uint32_t i = 0; i < links.size(); ++i) {
        links[i].setId(i);
    }
    /* Adjacency matrix */
    /* Init a graph of int value to calculate routes later*/
    Graph graph(nodes.size());
    Graph::initGraph(nodeMap, links, graph);

    Util util;
    for (int i = 0; i < flow_num; ++i) {
        node_idx a, b;
        do {
            a = util.getRandESIdx(esList);
            b = util.getRandESIdx(esList);
        } while (a == b);
        schedplus::PRIORITY_CODE_POINT pcp = util.getRandPCP();
        Flow flow(i,
                  Flow::getRandomPeriod(pcp),
                  pcp,
                  nodeMap.at(a),
                  nodeMap.at(b),
                  false,
                  1,
                  false);
        if (pcp == schedplus::P6) {
            /* unit: ns*/
            DeliveryGuarantee deliveryGuarantee(DDL, flow.getPeriod());
            flow.addDeliveryGuarantee(deliveryGuarantee);
        } else if (pcp == schedplus::P5) {
            /* Typically less than 90% of period. */
            /* unit: ns*/
            DeliveryGuarantee deliveryGuarantee(E2E, flow.getPeriod() / 10);
            flow.addDeliveryGuarantee(deliveryGuarantee);
        }
        flow.setFrameLength(Flow::getRandomFrameLength(pcp) + schedplus::HEADER_LEN);
        Util::calAllRoutes(nodeMap, flow, graph, links);
        flows.push_back(flow);
        oss.str("");
        flow.toString(oss);
        spdlog::get("console")->info("flow_{}: {}", i, oss.str());
    }
}


void openGACal(std::vector<Node *> &nodes,
               std::vector<Node *> &esList,
               std::vector<Node *> &swList,
               std::map<node_idx , Node *> &nodeMap,
               std::vector<DirectedLink> &links,
               std::vector<Flow> &flows) {
    std::ostringstream oss;
    /* End systems */
    Node *es00 = createNode(END_SYSTEM, "es00", 0);
    Node *es01 = createNode(END_SYSTEM, "es01", 0);
    Node *es02 = createNode(END_SYSTEM, "es02", 0);
    Node *es03 = createNode(END_SYSTEM, "es03", 0);
    Node *es04 = createNode(END_SYSTEM, "es04", 0);
    Node *es05 = createNode(END_SYSTEM, "es05", 0);
    Node *es06 = createNode(END_SYSTEM, "es06", 0);
    Node *es07 = createNode(END_SYSTEM, "es07", 0);
    Node *es08 = createNode(END_SYSTEM, "es08", 0);
    Node *es09 = createNode(END_SYSTEM, "es09", 0);
    Node *es10 = createNode(END_SYSTEM, "es10", 0);
    Node *es11 = createNode(END_SYSTEM, "es11", 0);
    Node *es12 = createNode(END_SYSTEM, "es12", 0);
    Node *es13 = createNode(END_SYSTEM, "es13", 0);
    Node *es14 = createNode(END_SYSTEM, "es14", 0);
    Node *es15 = createNode(END_SYSTEM, "es15", 0);
    Node *es16 = createNode(END_SYSTEM, "es16", 0);
    Node *es17 = createNode(END_SYSTEM, "es17", 0);


    /* Switches */
    Node *sw0 = createNode(SWITCH, "switch0", 30000);
    Node *sw1 = createNode(SWITCH, "switch1", 30000);
    Node *sw2 = createNode(SWITCH, "switch2", 30000);
    Node *sw3 = createNode(SWITCH, "switch3", 30000);
    Node *sw4 = createNode(SWITCH, "switch4", 30000);
    Node *sw5 = createNode(SWITCH, "switch5", 30000);
    Node *sw6 = createNode(SWITCH, "switch6", 30000);
    nodes.insert(nodes.end(), {es00, es01, es02, es03, es04, es05,
                               es06, es07, es08, es09, es10, es11,
                               es12, es13, es14, es15, es16, es17,
                               sw0, sw1, sw2, sw3, sw4, sw5, sw6});
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i]->setId(i);
    }
    esList.insert(esList.end(), {es00, es01, es02, es03, es04, es05,
                                 es06, es07, es08, es09, es10, es11,
                                 es12, es13, es14, es15, es16, es17});
    swList.insert(swList.end(), {sw0, sw1, sw2, sw3, sw4, sw5, sw6});
    for (node_idx i = 0; i < nodes.size(); ++i) {
        nodeMap[i] = nodes[i];
    }

    /* Links connected end systems to switches */
    FullDuplexLink link_00(es00, sw0, ((EndSystem *) es00)->getPort(), ((Switch *) sw0)->getPorts()[0]);
    FullDuplexLink link_01(es01, sw0, ((EndSystem *) es01)->getPort(), ((Switch *) sw0)->getPorts()[1]);
    FullDuplexLink link_02(es02, sw0, ((EndSystem *) es02)->getPort(), ((Switch *) sw0)->getPorts()[2]);

    FullDuplexLink link_03(es03, sw1, ((EndSystem *) es03)->getPort(), ((Switch *) sw1)->getPorts()[0]);
    FullDuplexLink link_04(es04, sw1, ((EndSystem *) es04)->getPort(), ((Switch *) sw1)->getPorts()[1]);
    FullDuplexLink link_05(es05, sw1, ((EndSystem *) es05)->getPort(), ((Switch *) sw1)->getPorts()[2]);

    FullDuplexLink link_06(es06, sw2, ((EndSystem *) es06)->getPort(), ((Switch *) sw2)->getPorts()[0]);
    FullDuplexLink link_07(es07, sw2, ((EndSystem *) es07)->getPort(), ((Switch *) sw2)->getPorts()[1]);
    FullDuplexLink link_08(es08, sw2, ((EndSystem *) es08)->getPort(), ((Switch *) sw2)->getPorts()[2]);

    FullDuplexLink link_09(es09, sw3, ((EndSystem *) es09)->getPort(), ((Switch *) sw3)->getPorts()[0]);
    FullDuplexLink link_10(es10, sw3, ((EndSystem *) es10)->getPort(), ((Switch *) sw3)->getPorts()[1]);
    FullDuplexLink link_11(es11, sw3, ((EndSystem *) es11)->getPort(), ((Switch *) sw3)->getPorts()[2]);

    FullDuplexLink link_12(es12, sw4, ((EndSystem *) es12)->getPort(), ((Switch *) sw4)->getPorts()[0]);
    FullDuplexLink link_13(es13, sw4, ((EndSystem *) es13)->getPort(), ((Switch *) sw4)->getPorts()[1]);
    FullDuplexLink link_14(es14, sw4, ((EndSystem *) es14)->getPort(), ((Switch *) sw4)->getPorts()[2]);

    FullDuplexLink link_15(es15, sw5, ((EndSystem *) es15)->getPort(), ((Switch *) sw5)->getPorts()[0]);
    FullDuplexLink link_16(es16, sw5, ((EndSystem *) es16)->getPort(), ((Switch *) sw5)->getPorts()[1]);
    FullDuplexLink link_17(es17, sw5, ((EndSystem *) es17)->getPort(), ((Switch *) sw5)->getPorts()[2]);

    /* Links connected four switches */
    FullDuplexLink link_18(sw0, sw6, ((Switch *) sw0)->getPorts()[3], ((Switch *) sw6)->getPorts()[0]);
    FullDuplexLink link_19(sw1, sw6, ((Switch *) sw1)->getPorts()[3], ((Switch *) sw6)->getPorts()[1]);
    FullDuplexLink link_20(sw2, sw6, ((Switch *) sw2)->getPorts()[3], ((Switch *) sw6)->getPorts()[2]);
    FullDuplexLink link_21(sw3, sw6, ((Switch *) sw3)->getPorts()[3], ((Switch *) sw6)->getPorts()[3]);
    FullDuplexLink link_22(sw4, sw6, ((Switch *) sw4)->getPorts()[3], ((Switch *) sw6)->getPorts()[4]);
    FullDuplexLink link_23(sw5, sw6, ((Switch *) sw5)->getPorts()[3], ((Switch *) sw6)->getPorts()[5]);

    FullDuplexLink link_24(sw0, sw1, ((Switch *) sw0)->getPorts()[4], ((Switch *) sw1)->getPorts()[5]);
    FullDuplexLink link_25(sw1, sw2, ((Switch *) sw1)->getPorts()[4], ((Switch *) sw2)->getPorts()[5]);
    FullDuplexLink link_26(sw2, sw3, ((Switch *) sw2)->getPorts()[4], ((Switch *) sw3)->getPorts()[5]);
    FullDuplexLink link_27(sw3, sw4, ((Switch *) sw3)->getPorts()[4], ((Switch *) sw4)->getPorts()[5]);
    FullDuplexLink link_28(sw4, sw5, ((Switch *) sw4)->getPorts()[4], ((Switch *) sw5)->getPorts()[5]);
    FullDuplexLink link_29(sw5, sw0, ((Switch *) sw5)->getPorts()[4], ((Switch *) sw0)->getPorts()[5]);

    /* Add links to vectors */
    links.insert(links.end(), {link_00.getLinks()[0], link_00.getLinks()[1],
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
                               link_15.getLinks()[0], link_15.getLinks()[1],
                               link_16.getLinks()[0], link_16.getLinks()[1],
                               link_17.getLinks()[0], link_17.getLinks()[1],
                               link_18.getLinks()[0], link_18.getLinks()[1],
                               link_19.getLinks()[0], link_19.getLinks()[1],
                               link_20.getLinks()[0], link_20.getLinks()[1],
                               link_21.getLinks()[0], link_21.getLinks()[1],
                               link_22.getLinks()[0], link_22.getLinks()[1],
                               link_23.getLinks()[0], link_23.getLinks()[1],
                               link_24.getLinks()[0], link_24.getLinks()[1],
                               link_25.getLinks()[0], link_25.getLinks()[1],
                               link_26.getLinks()[0], link_26.getLinks()[1],
                               link_27.getLinks()[0], link_27.getLinks()[1],
                               link_28.getLinks()[0], link_28.getLinks()[1],
                               link_29.getLinks()[0], link_29.getLinks()[1]});
    for (uint32_t i = 0; i < links.size(); ++i) {
        links[i].setId(i);
    }
    /* Adjacency matrix */
    /* Init a graph of int value to calculate routes later*/
    Graph graph(nodes.size());
    Graph::initGraph(nodeMap, links, graph);

    std::string flow_file_path = FLOW_FILE_LOCATION;
    flow_file_path.append("/flows.json");
    std::ifstream flow_file(flow_file_path);
    json jflows;
    flow_file >> jflows;
    for (json::iterator item_flow = jflows.begin(); item_flow != jflows.end(); ++item_flow) {
        uint32_t flow_id = (*item_flow)["id"].get<unsigned>();
        uint64_t offset = (*item_flow)["offset"].get<unsigned>();
        uint64_t period = (*item_flow)["period"].get<unsigned>();
        uint64_t length = (*item_flow)["length"].get<unsigned>();
        int pcp = ((*item_flow)["pcp"].get<int>());
        Flow flow(flow_id, offset, period, length, static_cast<schedplus::PRIORITY_CODE_POINT>(pcp));
        node_idx src = (*item_flow)["src"].get<unsigned>();
        node_idx dest = (*item_flow)["dest"].get<unsigned>();
        flow.setSrc(nodeMap.at(src));
        flow.setDest(nodeMap.at(dest));
        if (pcp == schedplus::P6) {
            /* unit: ns*/
            DeliveryGuarantee deliveryGuarantee(DDL, flow.getPeriod());
            flow.addDeliveryGuarantee(deliveryGuarantee);
        } else if (pcp == schedplus::P5) {
            /* Typically less than 90% of period. */
            /* unit: ns*/
            DeliveryGuarantee deliveryGuarantee(E2E, flow.getPeriod() / 10);
            flow.addDeliveryGuarantee(deliveryGuarantee);
        }
        Util::calAllRoutes(nodeMap, flow, graph, links);
        flows.push_back(flow);
        oss.str("");
        flow.toString(oss);
        spdlog::get("console")->info("flow_{}: {}", flow.getId(), oss.str());
    }
}


#endif //SCHEDPLUS_GA_H
