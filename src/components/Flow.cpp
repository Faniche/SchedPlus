//
// Created by faniche on 2022/1/25.
//

#include <sstream>
#include <random>
#include <spdlog/spdlog.h>
#include "Flow.h"
#include "Link.h"

const std::vector<uint8_t> Flow::randPeriod{1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 16, 18, 20};

/**
 * @param period                Period of frame
 * @param priorityCodePoint     PCP of flow
 * @param src                   Source of the flow
 * @param dest                  Destination of the flow
 * @param isCritical            The flow is time sensitive or not
 * @param rep                   The replication of the frame, default 1
 * @param multicast             The flow is multicast of not
 */
Flow::Flow(uint32_t _id, uint32_t period, PRIORITY_CODE_POINT priorityCodePoint, Node *src, Node *dest,
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

uint32_t Flow::getId() const {
    return id;
}

uint32_t Flow::getOffset() const {
    return offset;
}

void Flow::setOffset(uint32_t _offset) {
    Flow::offset = _offset;
}

/**
 * @brief return period of flow, unit: ns
 * @return flow period: ns
 **/
uint32_t Flow::getPeriod() const {
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

uint32_t Flow::getQueueDelay() const {
    return queueDelay;
}

void Flow::setQueueDelay(uint32_t _queueDelay) {
    Flow::queueDelay = _queueDelay;
}

uint16_t Flow::getFrameLength() const {
    return frameLength;
}

void Flow::setFrameLength(uint16_t _frameLength) {
    Flow::frameLength = _frameLength;
}

PRIORITY_CODE_POINT Flow::getPriorityCodePoint() const {
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
    oss << "\t" << R"("rep": )" << rep << "," << std::endl;
    oss << "\t" << R"("multicast": )" << std::boolalpha << multicast;
    if (routes.empty()) {
        oss << std::endl;
    } else {
        oss << "," << std::endl;
        oss << "\t" << R"("routes": {)";
        for (size_t i = 0; i < routes.size(); ++i) {
            oss << std::endl << "\t\t\"route" << i << "\": \"" << routes[i].getLinks()[0].get().getSrcNode()->getName();
            for (auto &j: routes[i].getLinks()) {
                oss << " -> " << j.get().getDestNode()->getName();
            }
            oss << "\",";
        }
        oss.seekp(-1, std::ios_base::end);
        oss << std::endl << "\t" << "}" << std::endl;
    }
    oss << "}" << std::endl;
    return oss.str();
}

uint32_t Flow::getHyperperiod() const {
    return hyperperiod;
}

void Flow::setHyperperiod(uint32_t _hyperperiod) {
    Flow::hyperperiod = _hyperperiod;
}

/**
 * @brief Calculate gate control list for every flows and check collision, the GCL is sorted by start time.
 * @return if the GCL has collision or not.
 * @retval  true:    GCL has no collision
 * @retval  false:   GCL has collision
 * */
bool Flow::addGateControlEntry(std::mutex &gcl_lock) {
    bool ret = true;
    if (this->priorityCodePoint == P5 || this->priorityCodePoint == P6) {
        uint32_t accumulatedDelay = this->offset;
        uint32_t prop_delay = 0, trans_delay = 0, proc_delay = 0;
        for (auto &link: routes[this->selectedRouteInx].getLinks()) {
            /* Add gate control entity. */
            proc_delay = link.get().getSrcNode()->getDpr();
            trans_delay = this->frameLength * link.get().getSrcPort().getMacrotick();
            accumulatedDelay += proc_delay;
            /* Add gate control entity for every frame of flow in a hycperperiod */
            uint32_t sendTimes = hyperperiod / period;
            spdlog::set_level(spdlog::level::info);
            spdlog::get("console")->debug("hyperperiod: {}, send {} times.", hyperperiod, sendTimes);
            for (int i = 0; i < sendTimes; ++i) {
                accumulatedDelay += offset + i * period;
                GateControlEntry gateControlEntry;
                gateControlEntry.setGateStatesValue(P6, GATE_OPEN);
                gateControlEntry.setStartTime(accumulatedDelay);
                gateControlEntry.setTimeIntervalValue(trans_delay);
                link.get().addGateControlEntry(gateControlEntry, gcl_lock);
            }
            accumulatedDelay += trans_delay;
            prop_delay = link.get().getLen() * link.get().getPropSpeed();
            accumulatedDelay += prop_delay;
            /* Sort the GCL of link */
            link.get().sortGCL(gcl_lock);
            /* Check collision */
            ret = link.get().checkGCLCollision();
        }
    }
    return ret;
}

/**
 * @brief Generate a period of frame in a flow. unit: ns
 * @retval uint32_t
 */
uint32_t Flow::getRandomPeriod(PRIORITY_CODE_POINT pcp) {
    if (pcp < 5) {
        spdlog::get("console")->error("Invalid PCP code: {}", pcp);
        return 0;
    }
    static std::random_device randomDevice;
    static std::default_random_engine engine(randomDevice());
    static std::uniform_int_distribution<uint8_t> randFramePeriod(0, randPeriod.size() - 1);
    uint8_t idx = randFramePeriod(engine);
    uint32_t a = randPeriod[idx];
    switch (pcp) {
        case P5:
            // 5                Cyclic                  2ms-20ms
            while (a == 1) a = randPeriod[randFramePeriod(engine)];
            //     μs     ns
            a *= (1000 * 1000);
            break;
        case P6:
            // 6                Isochronous             100μs-2ms
            //     μs    ns
            a *= (100 * 1000);
            break;
        case P7:
            // 7 (highest)      Network control         50ms-1s
            //          μs     ns
            a *= (50 * 1000 * 1000);
            break;
    }
    return a;
}

uint16_t Flow::getRandomFrameLength(PRIORITY_CODE_POINT pcp) {
    static std::random_device randomDevice;
    static std::default_random_engine engine(randomDevice());
    static std::uniform_int_distribution<uint16_t> randFrameLen;
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP0(300, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP1(1000, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP2(500, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP3(100, 1500);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP4(100, 200);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP5(50, 1000);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP6(30, 100);
    static std::uniform_int_distribution<uint16_t>::param_type paramFrameLenP7(50, 500);
    switch (pcp) {
        case P0:
            randFrameLen.param(paramFrameLenP0);
            break;
        case P1:
            randFrameLen.param(paramFrameLenP1);
            break;
        case P2:
            randFrameLen.param(paramFrameLenP2);
            break;
        case P3:
            randFrameLen.param(paramFrameLenP3);
            break;
        case P4:
            randFrameLen.param(paramFrameLenP4);
            break;
        case P5:
            randFrameLen.param(paramFrameLenP5);
            break;
        case P6:
            randFrameLen.param(paramFrameLenP6);
            break;
        case P7:
            randFrameLen.param(paramFrameLenP7);
            break;
    }
    uint16_t a = randFrameLen(engine);
    a -= a % 10;
    return a;
}

