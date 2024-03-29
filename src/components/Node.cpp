//
// Created by faniche on 2022/1/22.
//

#include <iostream>
#include <utility>
#include "Node.h"
#include "NodeImpl.h"

Node::Node(std::string _name, NODE_TYPE _nodeType, uint64_t _dpr) {
    this->name = std::move(_name);
    this->nodeType = _nodeType;
    this->dpr = _dpr;
}

Node::~Node() = default;


node_idx Node::getId() const {
    return id;
}

void Node::setId(node_idx _id) {
    Node::id = _id;
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

/**
 * @brief get precess delay of node, unit： ns
 * @return   uint64_t
 * */
uint64_t Node::getDpr() const {
    return dpr;
}

/**
 * @brief set precess delay of node.
 * @param   _dpr    process delay, unit： ns
 * */
void Node::setDpr(uint64_t _dpr) {
    Node::dpr = _dpr;
}

/**
 * @brief create a node pointer with given type
 * @param   nodeType    node type of the created node
 * @param   name        node name
 * @param   dpr         node process delay, unit: ns
 * @return  Node *
 * */
Node *createNode(NODE_TYPE nodeType, const std::string& name, uint64_t dpr){
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
