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
                hyperPeriod =lcm(a, b);
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
        return getRandInt(5, 6);
    }

    int getRandESIdx(std::vector<Node *> &esList) {
        return getRandInt(0, esList.size() - 1);
    }

    int getDDL(PRIORITY_CODE_POINT priority) {
        return 0;
    }
};

#endif //SCHEDPLUS_UTILS_H
