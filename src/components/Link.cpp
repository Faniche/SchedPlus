//
// Created by faniche on 2022/1/24.
//

#include <spdlog/spdlog.h>
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

Link *Link::nodesIdxToLink(const Node *nodeA, const Node *nodeB, std::vector<Link> &links) {
    spdlog::set_level(spdlog::level::debug);
    spdlog::debug("nodeA: {}, nodeB: {}", nodeA->getName(), nodeB->getName());
    for (size_t i = 0; i < links.size(); ++i) {
        spdlog::debug("comparing link: {}", i);
        if ((uuid_compare(links[i].getNodeA()->getId(), nodeA->getId()) == 0
             && uuid_compare(links[i].getNodeB()->getId(), nodeB->getId()) == 0)
            || (uuid_compare(links[i].getNodeA()->getId(), nodeB->getId()) == 0
                && uuid_compare(links[i].getNodeB()->getId(), nodeA->getId()) == 0)) {
            spdlog::debug("link mem address: {}", (void *)&links[i]);
            return &links[i];
        }
    }
    return nullptr;
}

