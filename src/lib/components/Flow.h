//
// Created by faniche on 2022/1/25.
//

#ifndef SCHEDPLUS_FLOW_H
#define SCHEDPLUS_FLOW_H


#include <uuid/uuid.h>
#include "NodeImpl.h"

class Flow {
private:
    uuid_t id{};

    /* The sending offset in a hyperperiod of each frame */
    int offset = 0;

    /* The interval between two frames of a Flow */
    int period = 0;

    /* The frame length include network headers of a packet of each frame */
    int frameLength = 0;

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

    std::vector<std::vector<Node>> routes;

public:
    /**
     * @param period        Period of frame.
     * @param frameLength   Frame of length
     * @param src           Source of the flow
     * @param dest          Destination of the flow
     * @param isCritical    The flow is time sensitive or not
     * @param rep           The replication of the frame, default 1
     * @param multicast     The flow is multicast of not
     */
    Flow(int period, int frameLength, Node *src, Node *dest, bool isCritical, int rep, bool multicast);

    std::string toString(std::ostringstream &oss);
};


#endif //SCHEDPLUS_FLOW_H
