//
// Created by faniche on 2022/1/24.
//

#ifndef SCHEDPLUS_LINK_H
#define SCHEDPLUS_LINK_H


#include <uuid/uuid.h>
#include <vector>
#include <mutex>
#include "Port.h"
#include "Node.h"

class DirectedLink {
private:
    uint32_t id;
    Node *srcNode;
    Node *destNode;
    Port srcPort;
    Port destPort;
    uint64_t speed = 1000000000;
    /* the length of link */
    uint16_t len = 20;
    /* the signal propagation speed of the media, val: nanosecond per meter */
    uint16_t propSpeed = 5;
public:

    DirectedLink();

    DirectedLink(Node *_srcNode, Node *_destNode, Port _srcPort, Port _destPort);

    DirectedLink(Node *_srcNode, Node *_destNode, Port _srcPort, Port _destPort, uint16_t _len, uint16_t _propSpeed);

    [[nodiscard]] uint32_t getId() const;

    void setId(uint32_t _id);

    [[nodiscard]] Node *getSrcNode() const;

    [[nodiscard]] Node *getDestNode() const;

    [[nodiscard]] const Port &getSrcPort() const;

    void addGateControlEntry(const GateControlEntry &gateControlEntry);

    void clearGateControlEntry();

    void sortGCL();

    bool checkGCLCollision();

    [[nodiscard]] const Port &getDestPort() const;

    [[nodiscard]] uint64_t getSpeed() const;

    [[nodiscard]] uint16_t getLen() const;

    void setLen(uint16_t len);

    [[nodiscard]] uint16_t getPropSpeed() const;

    void setPropSpeed(uint16_t propSpeed);

    static DirectedLink* nodesIdxToLink(const Node *_srcNode, const Node *_destNode, std::vector<DirectedLink> &links);
};

class FullDuplexLink {
private:
    std::vector<DirectedLink> links;
public:
    FullDuplexLink(Node *nodeA, Node *nodeB, const Port &_portA, const Port &_portB);

    [[nodiscard]] const std::vector<DirectedLink> &getLinks() const;
};

#endif //SCHEDPLUS_LINK_H
