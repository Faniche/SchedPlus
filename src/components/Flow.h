//
// Created by faniche on 2022/1/25.
//

#ifndef SCHEDPLUS_FLOW_H
#define SCHEDPLUS_FLOW_H


#include <uuid/uuid.h>
#include <mutex>
#include "NodeImpl.h"
#include "Link.h"
#include "../include/MyVlan.h"
#include "../Route.h"
#include "../Delivery/DeliveryGuarantee.h"

class Flow {
private:
    uint32_t id;

    /* The sending offset in a hyperperiod of each frame */
    uint32_t offset = 0;

    /* The interval between two frames of a Flow */
    uint32_t period = 0;

    uint32_t queueDelay = 0;

    /* The frame length include network headers of a packet of each frame */
    uint16_t frameLength = 0;

    PRIORITY_CODE_POINT priorityCodePoint;

    /* The delivery guarantee */
    std::vector<DeliveryGuarantee> deliveryGuarantees;

    /* The source node of a Flow */
    Node *src;

    /* The destination node of a Flow */
    Node *dest;

    /* the Flow is time-sensitive or not */
    bool isCritical = true;

    /* the Replication of frame */
    uint8_t rep = 1;

    /* the Flow is multicast or not */
    bool multicast = false;

    std::vector<Route> routes;

//    std::vector<std::vector<DirectedLink *>> routes;

    uint16_t selectedRouteInx = 0;

    uint32_t hyperperiod = 0;

    static const std::vector<uint8_t> randPeriod;

public:
    Flow(uint32_t _id, uint32_t period, PRIORITY_CODE_POINT priorityCodePoint, Node *src, Node *dest, bool isCritical, uint8_t rep, bool multicast);

    [[nodiscard]] uint32_t getId() const;

    [[nodiscard]] uint32_t getOffset() const;

    void setOffset(uint32_t offset);

    [[nodiscard]] uint32_t getPeriod() const;

    [[nodiscard]] const std::vector<DeliveryGuarantee> &getDeliveryGuarantees() const;

    void addDeliveryGuarantee(const DeliveryGuarantee &deliveryGuarantee);

    void setDeliveryGuarantee(const DeliveryGuarantee &deliveryGuarantee);

    void setDeliveryGuarantee();

    [[nodiscard]] uint32_t getQueueDelay() const;

    void setQueueDelay(uint32_t queueDelay);

    [[nodiscard]] uint16_t getFrameLength() const;

    void setFrameLength(uint16_t frameLength);

    [[nodiscard]] PRIORITY_CODE_POINT getPriorityCodePoint() const;

    [[nodiscard]] Node *getSrc() const;

    [[nodiscard]] Node *getDest() const;

    [[nodiscard]] bool isCritical1() const;

    [[nodiscard]] uint8_t getRep() const;

    [[nodiscard]] bool isMulticast() const;

    [[nodiscard]] const std::vector<Route> &getRoutes() const;

    void addRoutes(const Route &_route);

    [[nodiscard]] uint16_t getSelectedRouteInx() const;

    void setSelectedRouteInx(uint16_t selectedRouteInx);

    std::string toString(std::ostringstream &oss);

    [[nodiscard]] uint32_t getHyperperiod() const;

    bool addGateControlEntry(std::mutex &gcl_lock);

    void setHyperperiod(uint32_t hyperperiod);

    static uint32_t getRandomPeriod(PRIORITY_CODE_POINT pcp);

    static uint16_t getRandomFrameLength(PRIORITY_CODE_POINT pcp);

};


#endif //SCHEDPLUS_FLOW_H
