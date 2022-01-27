//
// Created by faniche on 2022/1/22.
//
#include "Node.h"
#include "NodeImpl.h"

#include <utility>

EndSystem::EndSystem(std::string name): Node(std::move(name), END_SYSTEM) {
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

Switch::Switch(std::string name): Node(std::move(name), SWITCH) {
    for (int i = 0; i < portNum; ++i) {
        Port port;
        ports.push_back(port);
    }
}

Switch::Switch(std::string name, int portNum) : Node(std::move(name), SWITCH), portNum(portNum) {
    for (int i = 0; i < portNum; ++i) {
        Port port;
        ports.push_back(port);
    }
}

int Switch::getPortNum() const {
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