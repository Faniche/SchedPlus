//
// Created by faniche on 2022/1/25.
//

#include <sstream>
#include <random>
#include <spdlog/spdlog.h>
#include "Flow.h"
#include "Link.h"

const std::vector<uint8_t> Flow::randPeriod{1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 16, 18, 20};

Flow::Flow(uint32_t _id, uint64_t offset, uint64_t period, uint64_t length, schedplus::PRIORITY_CODE_POINT pcp) :
        id(_id),
        offset(offset),
        period(period),
        frameLength(length),
        priorityCodePoint(pcp){}

/**
 * @param period                Period of frame
 * @param priorityCodePoint     PCP of flow
 * @param src                   Source of the flow
 * @param dest                  Destination of the flow
 * @param isCritical            The flow is time sensitive or not
 * @param rep                   The replication of the frame, default 1
 * @param multicast             The flow is multicast of not
 */
Flow::Flow(uint32_t _id, uint64_t period, schedplus::PRIORITY_CODE_POINT priorityCodePoint, Node *src, Node *dest,
           bool isCritical, uint8_t rep, bool multicast)
        : id(_id),
          period(period),
          priorityCodePoint(priorityCodePoint),
          src(src),
          dest(dest),
          isCritical(isCritical),
          rep(rep),
          multicast(multicast) {
//    DeliveryGuarantee deliveryGuarantee(E2E, 0);
//    deliveryGuarantees.emplace_back(deliveryGuarantee);
}

uint64_t Flow::getId() const {
    return id;
}

uint64_t Flow::getOffset() const {
    return offset;
}

void Flow::setOffset(uint64_t _offset) {
    Flow::offset = _offset;
}

/**
 * @brief return period of flow, unit: ns
 * @return flow period: ns
 **/
uint64_t Flow::getPeriod() const {
    return period;
}

const std::vector<DeliveryGuarantee> &Flow::getDeliveryGuarantees() const {
    return deliveryGuarantees;
}

void Flow::addDeliveryGuarantee(const DeliveryGuarantee &deliveryGuarantee) {
    this->deliveryGuarantees.push_back(deliveryGuarantee);
}

void Flow::setDeliveryGuarantee(const DeliveryGuarantee &deliveryGuarantee) {
    for (auto &i: deliveryGuarantees) {
        if (i.getType() == deliveryGuarantee.getType()) {
            i.setLowerVal(deliveryGuarantee.getLowerVal());
            i.setUpperVal(deliveryGuarantee.getUpperVal());
            i.setLoverObj(deliveryGuarantee.getLoverObj());
            i.setUpperObj(deliveryGuarantee.getUpperObj());
        }
    }
}

/**
 * @brief After set offset and route index, set guarantee.
 * */
void Flow::setDeliveryGuarantee() {
    for (auto &deliveryGuarantee: deliveryGuarantees) {
        if (deliveryGuarantee.getType() == DDL) {
            deliveryGuarantee.setLoverObj(offset + routes[selectedRouteInx].getE2E());
        } else if (deliveryGuarantee.getType() == E2E) {
            deliveryGuarantee.setLoverObj(queueDelay + routes[selectedRouteInx].getE2E());
        }
    }
}

uint64_t Flow::getQueueDelay() const {
    return queueDelay;
}

void Flow::setQueueDelay(uint64_t _queueDelay) {
    Flow::queueDelay = _queueDelay;
}

uint16_t Flow::getFrameLength() const {
    return frameLength;
}

void Flow::setFrameLength(uint16_t _frameLength) {
    Flow::frameLength = _frameLength;
}

schedplus::PRIORITY_CODE_POINT Flow::getPriorityCodePoint() const {
    return priorityCodePoint;
}


Node *Flow::getSrc() const {
    return src;
}

Node *Flow::getDest() const {
    return dest;
}

bool Flow::isCritical1() const {
    return isCritical;
}

uint8_t Flow::getRep() const {
    return rep;
}

bool Flow::isMulticast() const {
    return multicast;
}

const std::vector<Route> &Flow::getRoutes() const {
    return routes;
}

void Flow::addRoutes(const Route &_route) {
    routes.push_back(_route);
}

uint16_t Flow::getSelectedRouteInx() const {
    return selectedRouteInx;
}

void Flow::setSelectedRouteInx(uint16_t _selectedRouteInx) {
    Flow::selectedRouteInx = _selectedRouteInx;
}

std::string Flow::toString(std::ostringstream &oss) {
    oss << "{\n";
    oss << "\t" << R"("id": )" << id << "," << std::endl;
    oss << "\t" << R"("offset": )" << offset << "," << std::endl;
    oss << "\t" << R"("period": )" << period << "," << std::endl;
    oss << "\t" << R"("length": )" << frameLength << "," << std::endl;
    oss << "\t" << R"("priorityCodePoint": )" << priorityCodePoint << "," << std::endl;
    oss << "\t" << R"("srcNode": ")" << src->getName() << "," << std::endl;
    oss << "\t" << R"("destNode": ")" << dest->getName() << "," << std::endl;
    oss << "\t" << R"("isCritical": )" << std::boolalpha << isCritical << "," << std::endl;
    oss << "\t" << R"("rep": )" << std::to_string(rep) << "," << std::endl;
    oss << "\t" << R"("multicast": )" << std::boolalpha << multicast;
    if (routes.empty()) {
        oss << std::endl;
    } else {
        oss << "," << std::endl;
        oss << "\t" << R"("routes": {)";
        for (size_t i = 0; i < routes.size(); ++i) {
            oss << std::endl << "\t\t\"route" << i << "\": \"" << routes[i].toString();
            oss << "\",";
        }
        oss.seekp(-1, std::ios_base::end);
        oss << std::endl << "\t" << "}" << std::endl;
    }
    oss << "}" << std::endl;
    return oss.str();
}


/**
 * @brief Generate a period of frame in a flow. unit: ns
 * @retval uint32_t
 */
uint64_t Flow::getRandomPeriod(schedplus::PRIORITY_CODE_POINT pcp) {
    if (pcp < 5) {
        spdlog::get("console")->error("Invalid PCP code: {}", pcp);
        return 0;
    }
    static std::random_device randomDevice;
    static std::default_random_engine engine(randomDevice());
    static std::uniform_int_distribution<uint8_t> randFramePeriod(0, randPeriod.size() - 1);
    uint8_t idx = randFramePeriod(engine);
    uint64_t a = randPeriod[idx];
    switch (pcp) {
        case schedplus::P5:
            // 5                Cyclic                  2ms-20ms
            while (a == 1) a = randPeriod[randFramePeriod(engine)];
            //     μs     ns
            a *= (1000 * 1000);
            break;
        case schedplus::P6:
            // 6                Isochronous             100μs-2ms
            //     μs    ns
            a *= (100 * 1000);
            break;
        case schedplus::P7:
            // 7 (highest)      Network control         50ms-1s
            //          μs     ns
            a *= (50 * 1000 * 1000);
            break;
    }
    return a;
}

uint16_t Flow::getRandomFrameLength(schedplus::PRIORITY_CODE_POINT pcp) {
    static std::random_device randomDevice;
    static std::default_random_engine engine(randomDevice());
    static std::uniform_int_distribution<uint16_t> randFrameLen;
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP0(300, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP1(1000, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP2(500, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP3(100, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP4(100, 200);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP5(50, 1000);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP6(40, 100);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP7(50, 500);
    switch (pcp) {
        case schedplus::P0:
            randFrameLen.param(paramFrameLenP0);
            break;
        case schedplus::P1:
            randFrameLen.param(paramFrameLenP1);
            break;
        case schedplus::P2:
            randFrameLen.param(paramFrameLenP2);
            break;
        case schedplus::P3:
            randFrameLen.param(paramFrameLenP3);
            break;
        case schedplus::P4:
            randFrameLen.param(paramFrameLenP4);
            break;
        case schedplus::P5:
            randFrameLen.param(paramFrameLenP5);
            break;
        case schedplus::P6:
            randFrameLen.param(paramFrameLenP6);
            break;
        case schedplus::P7:
            randFrameLen.param(paramFrameLenP7);
            break;
    }
    uint16_t a = randFrameLen(engine);
    a -= a % 10;
    return a;
}

void Flow::setSrc(Node *_src) {
    Flow::src = _src;
}

void Flow::setDest(Node *_dest) {
    Flow::dest = _dest;
}

