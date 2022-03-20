//
// Created by faniche on 2022/1/25.
//

#ifndef SCHEDPLUS_FLOW_H
#define SCHEDPLUS_FLOW_H


#include <uuid/uuid.h>
#include "NodeImpl.h"
#include "Link.h"
#include "../include/MyVlan.h"

class Flow {
private:
    uuid_t id{};

    /* The sending offset in a hyperperiod of each frame */
    int offset = 0;

    /* The interval between two frames of a Flow */
    int period = 0;

    /* The deadline of arrival of frames */
    int deadline = 0;

    /* The frame length include network headers of a packet of each frame */
    int frameLength = 0;

    PRIORITY_CODE_POINT priorityCodePoint;

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

    std::vector<std::vector<DirectedLink *>> routes;

public:
    /**
     * @param period        Period of frame
     * @param deadline      deadline of frame
     * @param frameLength   Frame of length
     * @param src           Source of the flow
     * @param dest          Destination of the flow
     * @param isCritical    The flow is time sensitive or not
     * @param rep           The replication of the frame, default 1
     * @param multicast     The flow is multicast of not
     */
    Flow(int period, int deadline, int frameLength, Node *src, Node *dest, bool isCritical, int rep, bool multicast);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] int getOffset() const;

    void setOffset(int offset);

    [[nodiscard]] int getPeriod() const;

    [[nodiscard]] int getDeadline() const;

    [[nodiscard]] int getFrameLength() const;

    [[nodiscard]] PRIORITY_CODE_POINT getPriorityCodePoint() const;

    void setPriorityCodePoint(PRIORITY_CODE_POINT priorityCodePoint);

    [[nodiscard]] Node *getSrc() const;

    [[nodiscard]] Node *getDest() const;

    [[nodiscard]] bool isCritical1() const;

    [[nodiscard]] int getRep() const;

    [[nodiscard]] bool isMulticast() const;

    [[nodiscard]] const std::vector<std::vector<DirectedLink *>> &getRoutes() const;

    void setRoutes(const std::vector<DirectedLink *>& route);

    std::string toString(std::ostringstream &oss);
};


#endif //SCHEDPLUS_FLOW_H
