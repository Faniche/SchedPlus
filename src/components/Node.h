//
// Created by faniche on 2022/1/22.
//

#ifndef SCHEDPLUS_NODE_H
#define SCHEDPLUS_NODE_H

#include <string>
#include <map>
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

    [[nodiscard]] const std::string &getName() const;

    void setName(std::string& name);

    [[nodiscard]] NODE_TYPE getNodeType() const;

    virtual std::string& toString() = 0;

    static size_t nodeToIdx (const std::map<size_t, Node *> &map, const Node *node);
};

Node *createNode(NODE_TYPE nodeType, const std::string& name);

#endif //SCHEDPLUS_NODE_H