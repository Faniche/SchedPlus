//
// Created by faniche on 2022/1/24.
//

#include "Link.h"

Link::Link(Node *nodeA, Node *nodeB, const Port &portA, const Port &portB) : node_a(nodeA), node_b(nodeB),
                                                                             port_a(portA), port_b(portB) {
    uuid_generate(id);
}

const unsigned char *Link::getId() const {
    return id;
}

Node *Link::getNodeA() const {
    return node_a;
}

Node *Link::getNodeB() const {
    return node_b;
}

const Port &Link::getPortA() const {
    return port_a;
}

const Port &Link::getPortB() const {
    return port_b;
}

const Link *Link::nodesIdxToLink(const Node *nodeA, const Node *nodeB, const std::vector<Link> &links) {
    for (const auto & link : links) {
        if ((uuid_compare(link.getNodeA()->getId(), nodeA->getId()) == 0
             && uuid_compare(link.getNodeB()->getId(), nodeB->getId()) == 0)
            || (uuid_compare(link.getNodeA()->getId(), nodeB->getId()) == 0
                && uuid_compare(link.getNodeB()->getId(), nodeA->getId()) == 0)) {
            return &link;
        }
    }
    return nullptr;
}

