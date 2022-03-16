/**
 * File: Utils.h
 * Date: 2021/11/27
 * Author: Faniche
 */

//
// Created by faniche on 2021/11/27.
//

#ifndef Z3_SMT_UTILS_H
#define Z3_SMT_UTILS_H

#include <random>
#include <spdlog/spdlog.h>
#include <queue>
#include "components/Flow.h"
#include "Route.h"

//#define MIN_FRAME_LEN 64
//
//#define MAX_FRAME_LEN 1000
//
//#define MIN_FRAME_PERIOD 1
//
//#define MAX_FRAME_PERIOD 200

constexpr static int MIN_FRAME_LEN = 64;
constexpr static int MAX_FRAME_LEN = 1000;
constexpr static int MIN_FRAME_PERIOD = 0;
constexpr static int MAX_FRAME_PERIOD = 7;


/* https://stackoverflow.com/questions/3154454/what-is-the-most-efficient-way-to-calculate-the-least-common-multiple-of-two-int */
static long long gcd(long long int a, long long int b) {
    if (b == 0)
        return a;
    return gcd(b, a % b);
}

static long long lcm(long long a, long long b) {
    if (a > b)
        return (a / gcd(a, b)) * b;
    else
        return (b / gcd(a, b)) * a;
}

long long getHyperPeriod(std::vector<Flow> flows) {
    long long hyperPeriod, tmp = flows[0].getPeriod();
    for (size_t i = 1; i < flows.size(); ++i) {
        hyperPeriod = lcm(tmp, flows[i].getPeriod());
        tmp = hyperPeriod;
    }
    return hyperPeriod;
}

static int genRandInRange(int invokeIdx) {
    static std::random_device randomDevice;
    static std::default_random_engine engine(randomDevice());

    switch (invokeIdx) {
        case 0:
            static std::uniform_int_distribution<int> randPeriod(MIN_FRAME_PERIOD, MAX_FRAME_PERIOD);
            return randPeriod(engine);
        case 1:
            static std::uniform_int_distribution<int> randFrameLen(MIN_FRAME_LEN, MAX_FRAME_LEN);
            return randFrameLen(engine);
        default:
            return 0;
    }
}

int genRandFramePeriod() {
    int ret = (int) std::pow(2, genRandInRange(0));

    return ret;
}

int genRandFrameLen() {
    return genRandInRange(1);
}

int getWholeSendInterval(const std::vector<Flow> &flows) {
    int ret = 0;
    for (auto &flow: flows) {
        ret += flow.getFrameLength() * 24;
    }
    return ret;
}

void initGraph(const std::vector<Node *> &nodes,
               const std::vector<Link> &links,
               std::vector<std::vector<Node *>> &graph) {
    for (auto *node: nodes) {
        std::vector<Node *> tmp{node};
        graph.push_back(tmp);
    }
    for (auto &link: links) {
        for (auto &nodeList: graph) {
            if (nodeList[0] == link.getNodeA())
                nodeList.push_back(link.getNodeB());
            if (nodeList[0] == link.getNodeB())
                nodeList.push_back(link.getNodeA());
        }
    }
}

void printGraph(const std::vector<std::vector<Node *>> &graph) {
    for (auto &nodeList: graph) {
        std::string tmp;
        for (auto node = nodeList.begin(); node != nodeList.end(); node++) {
            tmp.append((*node)->getName());
            if (node < nodeList.end() - 1)
                tmp.append(" -> ");
        }
        spdlog::info(tmp);
    }
}

void initGraph(const std::vector<Node *> &nodes,
               const std::vector<Link> &links,
               std::vector<std::vector<GraphNode >> &nodesAdjacencyMatrix) {
    for (auto *node: nodes) {
        GraphNode graphNode(node->getId(), node->getName(), NOT_VISITED);
        std::vector<GraphNode> tmp{graphNode};
        nodesAdjacencyMatrix.push_back(tmp);
    }
    for (auto &link: links) {
        for (auto &nodeList: nodesAdjacencyMatrix) {
            if (uuid_compare(nodeList[0].getId(), link.getNodeA()->getId()) == 0) {
                GraphNode graphNode(link.getNodeB()->getId(), link.getNodeB()->getName(), NOT_VISITED);
                nodeList.push_back(graphNode);
            }
            if (uuid_compare(nodeList[0].getId(), link.getNodeB()->getId()) == 0) {
                GraphNode graphNode(link.getNodeA()->getId(), link.getNodeA()->getName(), NOT_VISITED);
                nodeList.push_back(graphNode);
            }
        }
    }
}

/**
 * @brief print the GraphNode type adjacency matrix
 * @param nodesAdjacencyMatrix  : GraphNode adjacency
 */
void printGraph(const std::vector<std::vector<GraphNode>> &nodesAdjacencyMatrix) {
    for (auto &nodeList: nodesAdjacencyMatrix) {
        std::string tmp;
        for (auto node = nodeList.begin(); node != nodeList.end(); node++) {
            tmp.append((*node).getName());
            if (node < nodeList.end() - 1)
                tmp.append(" -> ");
        }
        spdlog::info(tmp);
    }
}

/**
 * @brief Init the Integer type of graph
 * @param map       : reference to node index map
 * @param links     : link vector
 * @param graph     : integer adjacency matrix
 * */
void initGraph(std::map<size_t, Node *> &map, const std::vector<Link> &links, Graph &graph) {
    for (auto &link: links) {
        for (size_t i = 0; i < map.size(); ++i) {
            if (uuid_compare(map[i]->getId(), link.getNodeA()->getId()) == 0) {
                for (size_t j = 0; j < map.size(); ++j) {
                    if (uuid_compare(map[j]->getId(), link.getNodeB()->getId()) == 0)
                        graph.addEdge(i, j);
                }
            }
            if (uuid_compare(map[i]->getId(), link.getNodeB()->getId()) == 0) {
                for (size_t j = 0; j < map.size(); ++j) {
                    if (uuid_compare(map[j]->getId(), link.getNodeA()->getId()) == 0)
                        graph.addEdge(i, j);
                }
            }
        }
    }
}

/**
 * @brief Get all routes of flow
 * @param map   : node index map
 * @param flow  : flow waiting for calculation all routes
 * @param graph : Integer adjacency matrix of all node
 * @param links : links vector
 **/
void getRoutes(std::map<size_t, Node *> &map,
               Flow &flow,
               Graph &graph,
               std::vector<Link> &links) {
    size_t srcIdx = Node::nodeToIdx(map, flow.getSrc());
    if (srcIdx == INT64_MAX)
        spdlog::error("can not find the index of node: %s", flow.getSrc()->getName());
    size_t destIdx = Node::nodeToIdx(map, flow.getDest());
    if (destIdx == INT64_MAX)
        spdlog::error("can not find the index of node: %s", flow.getDest()->getName());
    std::vector<std::vector<size_t>> routes;
    graph.getAllRoutes(srcIdx, destIdx, routes);
    for (auto & idxRoute : routes) {
        srcIdx = idxRoute[0];
        std::vector<Link *> route;
        std::vector<Node *> nodeVector;
        nodeVector.push_back(flow.getSrc());
        for (int j = 1; j < idxRoute.size(); ++j) {
            destIdx = idxRoute[j];
            Link * link1 = Link::nodesIdxToLink(map.at(srcIdx), map.at(destIdx), links);
            nodeVector.push_back(map[destIdx]);
            srcIdx = destIdx;
            route.push_back(link1);
        }
        flow.setRoutes(route);
        flow.setRoutesAdj(nodeVector);
    }
}


#endif //Z3_SMT_UTILS_H
