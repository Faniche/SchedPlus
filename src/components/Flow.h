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
    uint64_t id;

    /* The sending offset in a hyperperiod of each frame */
    int offset = 0;

    /* The interval between two frames of a Flow */
    int period = 0;

    int queueDelay = 0;

    /* The frame length include network headers of a packet of each frame */
    int frameLength = 0;

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
    int rep = 1;

    /* the Flow is multicast or not */
    bool multicast = false;

    std::vector<Route> routes;

//    std::vector<std::vector<DirectedLink *>> routes;

    int selectedRouteInx = 0;

    uint64_t hyperperiod = 0;

    static const std::vector<int> randPeriod;

public:
    Flow(uint64_t _id, int period, PRIORITY_CODE_POINT priorityCodePoint, Node *src, Node *dest, bool isCritical, int rep, bool multicast);

    [[nodiscard]] uint64_t getId() const;

    [[nodiscard]] int getOffset() const;

    void setOffset(int offset);

    [[nodiscard]] int getPeriod() const;

    [[nodiscard]] const std::vector<DeliveryGuarantee> &getDeliveryGuarantees() const;

    void addDeliveryGuarantee(const DeliveryGuarantee &deliveryGuarantee);

    void setDeliveryGuarantee(const DeliveryGuarantee &deliveryGuarantee);

    void setDeliveryGuarantee();

    [[nodiscard]] int getQueueDelay() const;

    void setQueueDelay(int queueDelay);

    [[nodiscard]] int getFrameLength() const;

    void setFrameLength(int frameLength);

    [[nodiscard]] PRIORITY_CODE_POINT getPriorityCodePoint() const;

    [[nodiscard]] Node *getSrc() const;

    [[nodiscard]] Node *getDest() const;

    [[nodiscard]] bool isCritical1() const;

    [[nodiscard]] int getRep() const;

    [[nodiscard]] bool isMulticast() const;

    [[nodiscard]] const std::vector<Route> &getRoutes() const;

    void addRoutes(const Route &_route);

    [[nodiscard]] int getSelectedRouteInx() const;

    void setSelectedRouteInx(int selectedRouteInx);

    std::string toString(std::ostringstream &oss);

    [[nodiscard]] uint64_t getHyperperiod() const;

    bool addGateControlEntry(std::mutex &gcl_lock);

    void setHyperperiod(uint64_t hyperperiod);

    static int getRandomPeriod(PRIORITY_CODE_POINT pcp);

    static int getRandomFrameLength(PRIORITY_CODE_POINT pcp);

};


#endif //SCHEDPLUS_FLOW_H
