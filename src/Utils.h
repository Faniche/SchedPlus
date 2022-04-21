/**
 * File: Utils.h
 * Date: 2021/11/27
 * Author: Faniche
 */

//
// Created by faniche on 2021/11/27.
//

#ifndef SCHEDPLUS_UTILS_H
#define SCHEDPLUS_UTILS_H

#include <random>
#include <spdlog/spdlog.h>
#include <queue>
#include "components/Flow.h"
#include "Route.h"

class Util {
private:
    std::default_random_engine generator;
    std::uniform_int_distribution<int> randInteger;

public:
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

    /**
     * @brief unit: μs
     */
    static uint64_t getHyperPeriod(std::vector<Flow> &flows, std::ostringstream &oss) {
        oss.str("");
        long long hyperPeriod = 0, a = 0, b = 0;
        for (size_t i = 0; i < flows.size(); ++i) {
            if (flows[i].getPeriod() != 0) {
                if (a == 0) {
                    oss << "period: {" << flows[i].getPeriod();
                    a = flows[i].getPeriod() / 100;
                    continue;
                }
                b = flows[i].getPeriod() / 100;
                oss << ", " << flows[i].getPeriod();
                hyperPeriod = lcm(a, b);
                a = hyperPeriod;
            }
        }
        oss << "}";
        hyperPeriod *= 100;
        for (auto &flow: flows) {
            flow.setHyperperiod(hyperPeriod);
        }
        spdlog::set_level(spdlog::level::debug);
        spdlog::debug("Periods(unit: μs) of flows: {}, hyperperiod: {}μs", oss.str(), hyperPeriod);
        return hyperPeriod;
    }

    int getRandInt(size_t min, size_t max) {
        std::uniform_int_distribution<int>::param_type param(min, max);
        randInteger.param(param);
        return randInteger(generator);
    }

    PRIORITY_CODE_POINT getRandPCP() {
//        return getRandInt(0, 7);
        return static_cast<PRIORITY_CODE_POINT>(getRandInt(5, 6));
    }

    int getRandESIdx(std::vector<Node *> &esList) {
        return getRandInt(0, esList.size() - 1);
    }

    int getDDL(PRIORITY_CODE_POINT priority) {
        return 0;
    }

    /**
    * @brief Get all routes of flow
    * @param map   : node index map
    * @param flow  : flow waiting for calculation all routes
    * @param graph : Integer adjacency matrix of all node
    * @param alllinks : all alllinks vector
    **/
    void calAllRoutes(std::map<size_t, Node *> &map, Flow &flow, Graph &graph, std::vector<DirectedLink> &alllinks) {
        size_t srcIdx = Node::nodeToIdx(map, flow.getSrc());
        if (srcIdx == INT64_MAX)
            spdlog::error("can not find the index of node: %s", flow.getSrc()->getName());
        size_t destIdx = Node::nodeToIdx(map, flow.getDest());
        if (destIdx == INT64_MAX)
            spdlog::error("can not find the index of node: %s", flow.getDest()->getName());
        std::vector<std::vector<size_t>> routes;
        graph.getAllRoutes(srcIdx, destIdx, routes);
        for (auto &idxRoute: routes) {
            srcIdx = idxRoute[0];
            Route route;
            int tmp = 0;
            for (int j = 1; j < idxRoute.size(); ++j) {
                destIdx = idxRoute[j];
                std::reference_wrapper<DirectedLink> link1 = DirectedLink::nodesIdxToLink(map.at(srcIdx), map.at(destIdx), alllinks);
                srcIdx = destIdx;
                route.addLink(link1);
//                route.links.push_back(link1);
                /* Calculate the e2e latency except queue delay of flow. */
                int trans_delay = flow.getFrameLength() * link1.get().getSrcPort().getMacrotick();
                int proc_delay = link1.get().getSrcNode()->getDpr();
                int prop_delay = link1.get().getLen() * link1.get().getPropSpeed();
                tmp += trans_delay + proc_delay + prop_delay;
            }
            route.setE2E(tmp);
            flow.addRoutes(route);
        }

    }

};

#endif //SCHEDPLUS_UTILS_H
