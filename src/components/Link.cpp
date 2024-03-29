//
// Created by faniche on 2022/1/24.
//

#include "../../lib/spdlog/include/spdlog/spdlog.h"

#include <utility>
#include "Link.h"

DirectedLink::DirectedLink() {}

DirectedLink::DirectedLink(Node *_srcNode,
                           Node *_destNode,
                           Port _srcPort,
                           Port _destPort) : srcNode(_srcNode),
                                            destNode(_destNode),
                                            srcPort(std::move(_srcPort)),
                                            destPort(std::move(_destPort)) {}

DirectedLink::DirectedLink(Node *_srcNode,
                           Node *_destNode,
                           Port _srcPort,
                           Port _destPort,
                           uint16_t _len,
                           uint16_t _propSpeed) : srcNode(_srcNode),
                                             destNode(_destNode),
                                             srcPort(std::move(_srcPort)),
                                             destPort(std::move(_destPort)),
                                             len(_len),
                                             propSpeed(_propSpeed){}

uint32_t DirectedLink::getId() const {
    return id;
}

void DirectedLink::setId(uint32_t _id) {
    DirectedLink::id = _id;
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

void DirectedLink::addGateControlEntry(const GateControlEntry &gateControlEntry) {
    this->srcPort.addGateControlEntry(gateControlEntry);
}

void DirectedLink::clearGateControlEntry() {
    this->srcPort.clearGCL();
}

void DirectedLink::sortGCL() {
    this->srcPort.sortGCL();
}

void DirectedLink::mergeGCL() {
    this->srcPort.mergeGCL();
}

bool DirectedLink::checkGCLCollision() {
    return this->srcPort.checkGCLCollision();
}

const Port &DirectedLink::getDestPort() const {
    return destPort;
}

DirectedLink* DirectedLink::nodesIdxToLink(const Node *_srcNode, const Node *_destNode, std::vector<DirectedLink> &links) {
    spdlog::set_level(spdlog::level::info);
//    spdlog::get("console")->debug("_srcNode: %v, _destNode: %v", _srcNode->getName(), _destNode->getName());
    for (size_t i = 0; i < links.size(); ++i) {
        spdlog::get("console")->debug("comparing link: {}", i);
        if (links[i].getSrcNode()->getId() == _srcNode->getId()
            && links[i].getDestNode()->getId() == _destNode->getId()) {
            spdlog::get("console")->debug("link mem address: {}", (void *) &links[i]);
            return &links[i];
        }
    }
    return nullptr;
}

uint64_t DirectedLink::getSpeed() const {
    return speed;
}

uint16_t DirectedLink::getLen() const {
    return len;
}

void DirectedLink::setLen(uint16_t _len) {
    DirectedLink::len = _len;
}

uint16_t DirectedLink::getPropSpeed() const {
    return propSpeed;
}

void DirectedLink::setPropSpeed(uint16_t _propSpeed) {
    DirectedLink::propSpeed = _propSpeed;
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
