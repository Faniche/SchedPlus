//
// Created by faniche on 2022/1/24.
//

#ifndef SCHEDPLUS_LINK_H
#define SCHEDPLUS_LINK_H


#include <uuid/uuid.h>
#include <vector>
#include "Port.h"
#include "Node.h"

class Link {
private:
    uuid_t id{};
    Node *node_a;
    Node *node_b;
    Port port_a;
    Port port_b;
public:
    Link(Node *nodeA, Node *nodeB, const Port &portA, const Port &portB);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] Node *getNodeA() const;

    [[nodiscard]] Node *getNodeB() const;

    [[nodiscard]] const Port &getPortA() const;

    [[nodiscard]] const Port &getPortB() const;

    static Link * nodesIdxToLink(const Node *nodeA, const Node* nodeB, std::vector<Link> &links);
};


#endif //SCHEDPLUS_LINK_H
