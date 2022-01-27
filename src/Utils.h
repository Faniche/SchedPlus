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

//long long getHyperPeriod(std::vector<Flow> flows) {
//    long long hyperPeriod, tmp = flows[0].getPeriod();
//    for (size_t i = 1; i < flows.size(); ++i) {
//        hyperPeriod = lcm(tmp, flows[i].getPeriod());
//        tmp = hyperPeriod;
//    }
//    return hyperPeriod;
//}

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

//int getWholeSendInterval(const std::vector<Flow> &flows) {
//    int ret = 0;
//    for (auto &flow: flows) {
//        ret += flow.getFrameLength() * 24;
//    }
//    return ret;
//}

//std::vector<std::vector<Port>> getRoutes(const std::vector<Switch> &swList, const std::vector<EndSystem> &esList,
//                                         const EndSystem &src, const EndSystem &dest) {
//    std::vector<std::vector<Port>> routes;
//    for (int i = 0; i < ; ++i) {
//
//    }
//
//
//    return routes;
//}



#endif //Z3_SMT_UTILS_H
