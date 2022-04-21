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
    uuid_t id{};
    Node *srcNode;
    Node *destNode;
    Port srcPort;
    Port destPort;
    int speed = 1000000000;
    /* the length of link */
    int len = 20;
    /* the signal propagation speed of the media, val: nanosecond per meter */
    int propSpeed = 5;
public:

    DirectedLink();

    DirectedLink(Node *_srcNode, Node *_destNode, Port _srcPort, Port _destPort);

    DirectedLink(Node *_srcNode, Node *_destNode, Port _srcPort, Port _destPort, int _len, int _propSpeed);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] Node *getSrcNode() const;

    [[nodiscard]] Node *getDestNode() const;

    [[nodiscard]] const Port &getSrcPort() const;

    void addGateControlEntry(const GateControlEntry &gateControlEntry, std::mutex &gcl_lock);

    void sortGCL(std::mutex &gcl_lock);

    bool checkGCLCollision();

    [[nodiscard]] const Port &getDestPort() const;

    [[nodiscard]] int getSpeed() const;

    [[nodiscard]] int getLen() const;

    void setLen(int len);

    [[nodiscard]] int getPropSpeed() const;

    void setPropSpeed(int propSpeed);

    static std::reference_wrapper<DirectedLink> nodesIdxToLink(const Node *_srcNode, const Node *_destNode, std::vector<DirectedLink> &links);
};

class FullDuplexLink {
private:
    std::vector<DirectedLink> links;
public:
    FullDuplexLink(Node *nodeA, Node *nodeB, const Port &_portA, const Port &_portB);

    [[nodiscard]] const std::vector<DirectedLink> &getLinks() const;
};

#endif //SCHEDPLUS_LINK_H
