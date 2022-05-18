//
// Created by faniche on 2022/1/22.
//

#ifndef SCHEDPLUS_NODE_H
#define SCHEDPLUS_NODE_H

#include <string>
#include <map>
#include "Port.h"

typedef size_t node_idx;

enum NODE_TYPE {
    SWITCH,
    END_SYSTEM
};

class Node {
protected:
    uuid_t id{};
    std::string name;
    NODE_TYPE nodeType;
    /* the process delay of node: ns*/
    uint64_t dpr;

public:
    Node(std::string name, NODE_TYPE _nodeType, uint64_t _dpr);

    virtual ~Node();

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] const std::string &getName() const;

    void setName(std::string& name);

    [[nodiscard]] NODE_TYPE getNodeType() const;

    virtual std::string& toString() = 0;

    static size_t nodeToIdx (const std::map<size_t, Node *> &map, const Node *node);

    [[nodiscard]] uint64_t getDpr() const;

    void setDpr(uint64_t dpr);
};

Node *createNode(NODE_TYPE nodeType, const std::string& name, uint64_t dpr);

#endif //SCHEDPLUS_NODE_H
