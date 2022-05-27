//
// Created by faniche on 2022/1/22.
//

#ifndef SCHEDPLUS_NODEIMPL_H
#define SCHEDPLUS_NODEIMPL_H


#include <string>
#include <vector>
#include "Node.h"
#include "Port.h"

class EndSystem : public Node {
private:
    Port port;
public:

    explicit EndSystem(std::string name, uint64_t dpr);

    [[nodiscard]] const Port &getPort() const;

    void setPort(const Port &port);

    std::string &toString() override;
};

class Switch : public Node{
private:
    uint32_t portNum = 8;
    std::vector<Port> ports;
public:
    explicit Switch(std::string name, uint64_t dpr);

    Switch(std::string name, uint64_t dpr, uint32_t portNum);

    [[nodiscard]] uint32_t getPortNum() const;

    [[nodiscard]] const std::vector<Port> &getPorts() const;

    void setPorts(const std::vector<Port> &ports);

    std::string &toString() override;
};

enum NODE_SEARCH_STATE {
    NOT_VISITED,
    VISITING,
    VISITED
};

class GraphNode {
private:
    uuid_t id{};
    std::string name;
    NODE_SEARCH_STATE state = NOT_VISITED;
    uint8_t hop = INT8_MAX;
    uuid_t parent{};
public:

    GraphNode(const unsigned char *id, std::string name, NODE_SEARCH_STATE state);

    GraphNode(const GraphNode &graphNode);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] const std::string &getName() const;

    [[nodiscard]] NODE_SEARCH_STATE getState() const;

    void setState(NODE_SEARCH_STATE _state);

    [[nodiscard]] uint8_t getHop() const;

    void setHop(uint8_t hop);

    [[nodiscard]] const unsigned char *getParent() const;

    void setParent (const unsigned char *parent);
};

#endif //SCHEDPLUS_NODEIMPL_H
