//
// Created by faniche on 22-5-18.
//

#ifndef SCHEDPLUS_GA_4SW_RING_H
#define SCHEDPLUS_GA_4SW_RING_H

#include <vector>
#include <fstream>
#include "../components/Flow.h"
#include "../../lib/openGA.hpp"
#include "GA_Solution.h"
#include "MyFunctions.h"

class Small4SwRing {
public:
    static void openGACal(const size_t flow_num,
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
//        Node *es08 = createNode(END_SYSTEM, "es08", 0);
//        Node *es09 = createNode(END_SYSTEM, "es09", 0);
//        Node *es10 = createNode(END_SYSTEM, "es10", 0);
//        Node *es11 = createNode(END_SYSTEM, "es11", 0);

        /* Switches */
        Node *sw0 = createNode(SWITCH, "switch0", 30000);
        Node *sw1 = createNode(SWITCH, "switch1", 30000);
        Node *sw2 = createNode(SWITCH, "switch2", 30000);
        Node *sw3 = createNode(SWITCH, "switch3", 30000);
        nodes.insert(nodes.end(), {es00, es01, es02, es03, es04, es05,
                                  es06, es07,
                                  sw0, sw1, sw2, sw3});
        esList.insert(esList.end(), {es00, es01, es02, es03, es04, es05,
                                   es06, es07});
        swList.insert(swList.end(), {sw0, sw1, sw2, sw3});
        for (node_idx i = 0; i < nodes.size(); ++i) {
            nodeMap[i] = nodes[i];
        }

        /* Links connected end systems to switches */
        FullDuplexLink link_00(es00, sw0, ((EndSystem *) es00)->getPort(), ((Switch *) sw0)->getPorts()[0]);
        FullDuplexLink link_01(es01, sw0, ((EndSystem *) es01)->getPort(), ((Switch *) sw0)->getPorts()[1]);
        FullDuplexLink link_02(es02, sw1, ((EndSystem *) es02)->getPort(), ((Switch *) sw1)->getPorts()[0]);
        FullDuplexLink link_03(es03, sw1, ((EndSystem *) es03)->getPort(), ((Switch *) sw1)->getPorts()[1]);
        FullDuplexLink link_04(es04, sw2, ((EndSystem *) es04)->getPort(), ((Switch *) sw2)->getPorts()[0]);
        FullDuplexLink link_05(es05, sw2, ((EndSystem *) es05)->getPort(), ((Switch *) sw2)->getPorts()[1]);
        FullDuplexLink link_06(es06, sw3, ((EndSystem *) es06)->getPort(), ((Switch *) sw3)->getPorts()[0]);
        FullDuplexLink link_07(es07, sw3, ((EndSystem *) es07)->getPort(), ((Switch *) sw3)->getPorts()[1]);

        /* Links connected four switches */
        FullDuplexLink link_24(sw0, sw1, ((Switch *) sw0)->getPorts()[2], ((Switch *) sw1)->getPorts()[3]);
        FullDuplexLink link_25(sw1, sw2, ((Switch *) sw1)->getPorts()[2], ((Switch *) sw2)->getPorts()[3]);
        FullDuplexLink link_26(sw2, sw3, ((Switch *) sw2)->getPorts()[2], ((Switch *) sw3)->getPorts()[3]);
        FullDuplexLink link_27(sw3, sw0, ((Switch *) sw3)->getPorts()[2], ((Switch *) sw0)->getPorts()[3]);

        /* Add links to vectors */
        links.insert(links.end(), {link_00.getLinks()[0], link_00.getLinks()[1],
                                        link_01.getLinks()[0], link_01.getLinks()[1],
                                        link_02.getLinks()[0], link_02.getLinks()[1],
                                        link_03.getLinks()[0], link_03.getLinks()[1],
                                        link_04.getLinks()[0], link_04.getLinks()[1],
                                        link_05.getLinks()[0], link_05.getLinks()[1],
                                        link_06.getLinks()[0], link_06.getLinks()[1],
                                        link_07.getLinks()[0], link_07.getLinks()[1],
                                        link_24.getLinks()[0], link_24.getLinks()[1],
                                        link_25.getLinks()[0], link_25.getLinks()[1],
                                        link_26.getLinks()[0], link_26.getLinks()[1],
                                        link_27.getLinks()[0], link_27.getLinks()[1]});
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
            a = i % 8;
            b = (a + 3)  % 8;
//            do {
//                a = util.getRandESIdx(esList);
//                b = util.getRandESIdx(esList);
//            } while (a == b);
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
            util.calAllRoutes(nodeMap, flow, graph, links);
            flows.push_back(flow);
            oss.str("");
            flow.toString(oss);
            spdlog::get("console")->info("flow_{}: {}", i, oss.str());
        }
    }
};


#endif //SCHEDPLUS_GA_4SW_RING_H
