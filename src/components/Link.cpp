//
// Created by faniche on 2022/1/24.
//

#include <spdlog/spdlog.h>
#include "Link.h"

DirectedLink::DirectedLink(Node *_srcNode, Node *_destNode, const Port &_srcPort, const Port &_destPort) : srcNode(_srcNode), destNode(_destNode),
                                                                                                           srcPort(_srcPort), destPort(_destPort) {
    uuid_generate(id);
}

const unsigned char *DirectedLink::getId() const {
    return id;
}

Node *DirectedLink::getSrcNode() const {
    return srcNode;
}

Node *DirectedLink::getDestNode() const {
    return destNode;
}

const Port &DirectedLink::getSrcPort() const {
    return srcPort;
}

const Port &DirectedLink::getDestPort() const {
    return destPort;
}

DirectedLink *DirectedLink::nodesIdxToLink(const Node *_srcNode, const Node *_destNode, std::vector<DirectedLink> &links) {
    spdlog::set_level(spdlog::level::debug);
    spdlog::debug("_srcNode: {}, _destNode: {}", _srcNode->getName(), _destNode->getName());
    for (size_t i = 0; i < links.size(); ++i) {
        spdlog::debug("comparing link: {}", i);
        if (uuid_compare(links[i].getSrcNode()->getId(), _srcNode->getId()) == 0
             && uuid_compare(links[i].getDestNode()->getId(), _destNode->getId()) == 0) {
            spdlog::debug("link mem address: {}", (void *)&links[i]);
            return &links[i];
        }
    }
    return nullptr;
}

FullDuplexLink::FullDuplexLink(Node *nodeA, Node *nodeB, const Port &_portA, const Port &_portB) {
    DirectedLink link1(nodeA, nodeB, _portA, _portB);
    DirectedLink link2(nodeB, nodeA, _portB, _portA);
    links.push_back(link1);
    links.push_back(link2);
}

const std::vector<DirectedLink> &FullDuplexLink::getLinks() const {
    return links;
}
