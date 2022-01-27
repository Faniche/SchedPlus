//
// Created by faniche on 2022/1/22.
//

#ifndef SCHEDPLUS_NODE_H
#define SCHEDPLUS_NODE_H

#include <string>
#include "Port.h"

enum NODE_TYPE {
    SWITCH,
    END_SYSTEM
};

class Node {
protected:
    uuid_t id{};
    std::string name;
    NODE_TYPE nodeType;

public:
    Node(std::string name, NODE_TYPE _nodeType);

    virtual ~Node();

    [[nodiscard]] const unsigned char *getId() const;

    std::string &getName();

    void setName(std::string& name);

    [[nodiscard]] NODE_TYPE getNodeType() const;

    virtual std::string& toString() = 0;
};

Node *createNode(NODE_TYPE nodeType, const std::string& name);

#endif //SCHEDPLUS_NODE_H
