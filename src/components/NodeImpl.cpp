//
// Created by faniche on 2022/1/22.
//
#include "Node.h"
#include "NodeImpl.h"

#include <utility>

EndSystem::EndSystem(std::string name, uint32_t dpr): Node(std::move(name), END_SYSTEM, dpr) {
    Port _port;
    this->port = _port;
}

const Port &EndSystem::getPort() const {
    return port;
}

void EndSystem::setPort(const Port &_port) {
    this->port = _port;
}

std::string &EndSystem::toString() {
    return this->name;
}

Switch::Switch(std::string name, uint32_t dpr): Node(std::move(name), SWITCH, dpr) {
    for (int i = 0; i < portNum; ++i) {
        Port port;
        ports.push_back(port);
    }
}

Switch::Switch(std::string name, uint32_t dpr, uint32_t portNum) : Node(std::move(name), SWITCH, dpr), portNum(portNum) {
    for (int i = 0; i < portNum; ++i) {
        Port port;
        ports.push_back(port);
    }
}

uint32_t Switch::getPortNum() const {
    return portNum;
}

const std::vector<Port> &Switch::getPorts() const {
    return ports;
}

void Switch::setPorts(const std::vector<Port> &_ports) {
    this->ports = _ports;
}

std::string &Switch::toString() {
    return this->name;
}


GraphNode::GraphNode(const unsigned char *_id, std::string name, NODE_SEARCH_STATE state) : name(std::move(name)),
                                                                                                  state(state) {
    uuid_copy(id, _id);
}

GraphNode::GraphNode(const GraphNode &graphNode) {
    uuid_copy(this->id, graphNode.id);
    this->name = graphNode.name;
    this->state = graphNode.state;
    this->hop = graphNode.hop;
    uuid_copy(this->parent, graphNode.parent);
}

const unsigned char *GraphNode::getId() const {
    return id;
}

const std::string &GraphNode::getName() const {
    return name;
}

NODE_SEARCH_STATE GraphNode::getState() const {
    return state;
}

void GraphNode::setState(NODE_SEARCH_STATE _state) {
    GraphNode::state = _state;
}

uint8_t GraphNode::getHop() const {
    return hop;
}

void GraphNode::setHop(uint8_t _hop) {
    GraphNode::hop = _hop;
}

const unsigned char *GraphNode::getParent() const {
    return parent;
}

void GraphNode::setParent(const unsigned char *_parent) {
    uuid_copy(parent, _parent);
}