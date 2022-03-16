//
// Created by faniche on 2022/1/24.
//

#ifndef SCHEDPLUS_LINK_H
#define SCHEDPLUS_LINK_H


#include <uuid/uuid.h>
#include <vector>
#include "Port.h"
#include "Node.h"

class DirectedLink {
private:
    uuid_t id{};
    Node *srcNode;
    Node *destNode;
    Port srcPort;
    Port destPort;
public:
    DirectedLink(Node *_srcNode, Node *_destNode, const Port &_srcPort, const Port &_destPort);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] Node *getSrcNode() const;

    [[nodiscard]] Node *getDestNode() const;

    [[nodiscard]] const Port &getSrcPort() const;

    [[nodiscard]] const Port &getDestPort() const;

    static DirectedLink *nodesIdxToLink(const Node *_srcNode, const Node *_destNode, std::vector<DirectedLink> &links);
};

class FullDuplexLink {
private:
    std::vector<DirectedLink> links;
public:
    FullDuplexLink(Node *nodeA, Node *nodeB, const Port &_portA, const Port &_portB);

    [[nodiscard]] const std::vector<DirectedLink> &getLinks() const;
};

#endif //SCHEDPLUS_LINK_H
