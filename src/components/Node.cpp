//
// Created by faniche on 2022/1/22.
//

#include <iostream>
#include <utility>
#include "Node.h"
#include "NodeImpl.h"

Node::Node(std::string _name, NODE_TYPE _nodeType, int _dpr) {
    uuid_generate(this->id);
    this->name = std::move(_name);
    this->nodeType = _nodeType;
    this->dpr = _dpr;
}

Node::~Node() = default;

const unsigned char *Node::getId() const {
    return id;
}

const std::string &Node::getName() const {
    return name;
}

void Node::setName(std::string &_name) {
    this->name = _name;
}

NODE_TYPE Node::getNodeType() const {
    return nodeType;
}

size_t Node::nodeToIdx(const std::map<size_t, Node *> &map, const Node *node) {
    for (const auto&[key, value]: map) {
        if (uuid_compare(node->getId(), value->getId()) == 0) {
            return key;
        }
    }
    return INT64_MAX;
}

int Node::getDpr() const {
    return dpr;
}

void Node::setDpr(int _dpr) {
    Node::dpr = _dpr;
}

Node *createNode(NODE_TYPE nodeType, const std::string& name, int dpr){
    Node *node;
    switch (nodeType) {
        case END_SYSTEM:
            node = new EndSystem(name, dpr);
            break;
        case SWITCH:
            node = new Switch(name, dpr);
            break;
        default:
            std::cout << "wrong components type" << std::endl;
            node = nullptr;
    }
    return node;
}
