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
#include "lib/components/Flow.h"

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

std::vector<std::vector<Link>> getRoutes(const std::vector<Switch> &swList,
                                         const std::vector<EndSystem> &esList,
                                         const EndSystem &src,
                                         const EndSystem &dest) {
    std::vector<std::vector<Link>> routes;
    for (int i = 0; i < routes.size(); ++i) {

    }
    return routes;
}

//std::vector<std::vector<Link>> getRoutes(const std::vector<Link> &links,
//                                         std::vector<std::vector<GraphNode>>& nodesAdjacencyMatrix,
//                                         const Node &src,
//                                         const Node &dest) {
//    std::vector<std::vector<Link>> routes;
//    std::queue<GraphNode> nodeQueue;
//    for (auto & nodeList: nodesAdjacencyMatrix) {
//        if (uuid_compare(nodeList[0].getId(), src.getId()) == 0) {
//            nodeList[0].setHop(0);
//            nodeQueue.push(nodeList[0]);
//            break;
//        }
//    }
//    while (!nodeQueue.empty()) {
//        GraphNode tmp = nodeQueue.back();
//        for (auto &nodeList: nodesAdjacencyMatrix) {
//            if (uuid_compare(nodeList[0].getId(), nodeQueue.back().getId()) == 0) {
//                /* Iterate the adjacency matrix of tmp */
//                for (auto & node : nodeList) {
//                    if (node.getState() == NOT_VISITED) {
//                        node.setState(VISITING);
//                        node.setHop(nodeQueue.back().getHop() + 1);
//                        node.setParent(nodeQueue.back().getId());
//                        nodeQueue.push(node);
//                    }
//                    if (uuid_compare(node.getId(), dest.getId()) == 0) {
//                        spdlog::info("Got");
//                    }
//                }
//                nodeQueue.back().setState(VISITED);
//                nodeQueue.pop();
//            }
//        }
//
//    }
//    spdlog::info("Calculate routes finished");
//    return routes;
//}

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

void printGraph(const std::vector<std::vector<Node *>> &graph){
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

void printGraph(const std::vector<std::vector<GraphNode>> &nodesAdjacencyMatrix){
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

#endif //Z3_SMT_UTILS_H
