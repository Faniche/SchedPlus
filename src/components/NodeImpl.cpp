//
// Created by faniche on 2022/1/22.
//
#include "Node.h"
#include "NodeImpl.h"

#include <utility>

EndSystem::EndSystem(std::string name, uint64_t dpr): Node(std::move(name), END_SYSTEM, dpr) {
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

Switch::Switch(std::string name, uint64_t dpr): Node(std::move(name), SWITCH, dpr) {
    for (int i = 0; i < portNum; ++i) {
        Port port;
        ports.push_back(port);
    }
}

Switch::Switch(std::string name, uint64_t dpr, uint32_t portNum) : Node(std::move(name), SWITCH, dpr), portNum(portNum) {
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
