//
// Created by faniche on 22-5-19.
//

#ifndef SCHEDPLUS_GA_LINE_2_1_H
#define SCHEDPLUS_GA_LINE_2_1_H

#include <vector>
#include <fstream>
#include "../components/Flow.h"
#include "../../lib/openGA/openGA.hpp"
#include "../../lib/json/json.hpp"
#include "GA_Solution.h"

using json = nlohmann::json;

class GA_line_2_1 {
public:
    static void openGACal(const size_t flow_num,
                          std::vector<Node *> &nodes,
                          std::vector<Node *> &esList,
                          std::vector<Node *> &swList,
                          std::map<node_idx, Node *> &nodeMap,
                          std::vector<DirectedLink> &links,
                          std::vector<Flow> &flows) {
        std::ostringstream oss;
        /* End systems */
        Node *es00 = createNode(END_SYSTEM, "es00", 0);
        Node *es01 = createNode(END_SYSTEM, "es01", 0);

        /* Switches */
        Node *sw0 = createNode(SWITCH, "switch0", 30000);
        nodes.insert(nodes.end(), {es00, es01, sw0});
        for (int i = 0; i < nodes.size(); ++i) {
            nodes[i]->setId(i);
        }
        esList.insert(esList.end(), {es00, es01});
        swList.insert(swList.end(), {sw0});
        for (node_idx i = 0; i < nodes.size(); ++i) {
            nodeMap[i] = nodes[i];
        }

        /* Links connected end systems to switches */
        FullDuplexLink link_00(es00, sw0, ((EndSystem *) es00)->getPort(), ((Switch *) sw0)->getPorts()[0]);
        FullDuplexLink link_01(es01, sw0, ((EndSystem *) es01)->getPort(), ((Switch *) sw0)->getPorts()[1]);

        /* Add links to vectors */
        links.insert(links.end(), {link_00.getLinks()[0], link_00.getLinks()[1],
                                   link_01.getLinks()[0], link_01.getLinks()[1]});
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
            if (i <= flow_num / 2) {
                a = 0;
                b = 1;
            } else {
                a = 1;
                b = 0;
            }
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

    static void openGACal(std::vector<Node *> &nodes,
                          std::vector<Node *> &esList,
                          std::vector<Node *> &swList,
                          std::map<node_idx, Node *> &nodeMap,
                          std::vector<DirectedLink> &links,
                          std::vector<Flow> &flows) {
        std::ostringstream oss;
        /* End systems */
        Node *es00 = createNode(END_SYSTEM, "es00", 0);
        Node *es01 = createNode(END_SYSTEM, "es01", 0);

        /* Switches */
        Node *sw0 = createNode(SWITCH, "switch0", 30000);
        nodes.insert(nodes.end(), {es00, es01, sw0});
        for (int i = 0; i < nodes.size(); ++i) {
            nodes[i]->setId(i);
        }
        esList.insert(esList.end(), {es00, es01});
        swList.insert(swList.end(), {sw0});
        for (node_idx i = 0; i < nodes.size(); ++i) {
            nodeMap[i] = nodes[i];
        }

        /* Links connected end systems to switches */
        FullDuplexLink link_00(es00, sw0, ((EndSystem *) es00)->getPort(), ((Switch *) sw0)->getPorts()[0]);
        FullDuplexLink link_01(es01, sw0, ((EndSystem *) es01)->getPort(), ((Switch *) sw0)->getPorts()[1]);

        /* Add links to vectors */
        links.insert(links.end(), {link_00.getLinks()[0], link_00.getLinks()[1],
                                   link_01.getLinks()[0], link_01.getLinks()[1]});
        for (uint32_t i = 0; i < links.size(); ++i) {
            links[i].setId(i);
        }
        /* Adjacency matrix */
        /* Init a graph of int value to calculate routes later*/
        Graph graph(nodes.size());
        Graph::initGraph(nodeMap, links, graph);

        Util util;

        std::string flow_file_path = FLOW_FILE_LOCATION;
        flow_file_path.append("/flows.json");
        std::ifstream flow_file(flow_file_path);
        json jflows;
        flow_file >> jflows;
        std::cout << jflows.dump(4) << std::endl;
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
};


#endif //SCHEDPLUS_GA_LINE_2_1_H
