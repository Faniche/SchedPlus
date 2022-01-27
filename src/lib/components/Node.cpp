//
// Created by faniche on 2022/1/22.
//

#include <iostream>
#include <utility>
#include "Node.h"
#include "NodeImpl.h"

Node::Node(std::string _name, NODE_TYPE _nodeType) {
    uuid_generate(this->id);
    this->name = std::move(_name);
    this->nodeType = _nodeType;
}

Node::~Node() = default;

const unsigned char *Node::getId() const {
    return id;
}

std::string &Node::getName() {
    return this->name;
}

void Node::setName(std::string &_name) {
    this->name = _name;
}

NODE_TYPE Node::getNodeType() const {
    return nodeType;
}

Node *createNode(NODE_TYPE nodeType, const std::string& name){
    Node *node;
    switch (nodeType) {
        case END_SYSTEM:
            node = new EndSystem(name);
            break;
        case SWITCH:
            node = new Switch(name);
            break;
        default:
            std::cout << "wrong components type" << std::endl;
            node = nullptr;
    }
    return node;
}
