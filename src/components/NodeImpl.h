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
    explicit EndSystem(std::string name);

    [[nodiscard]] const Port &getPort() const;

    void setPort(const Port &port);

    std::string &toString() override;
};

class Switch : public Node{
private:
    int portNum = 8;
    std::vector<Port> ports;
public:
    explicit Switch(std::string name);

    Switch(std::string name, int portNum);

    [[nodiscard]] int getPortNum() const;

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
    int hop = INT32_MAX;
    uuid_t parent{};
public:

    GraphNode(const unsigned char *id, std::string name, NODE_SEARCH_STATE state);

    GraphNode(const GraphNode &graphNode);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] const std::string &getName() const;

    [[nodiscard]] NODE_SEARCH_STATE getState() const;

    void setState(NODE_SEARCH_STATE _state);

    [[nodiscard]] int getHop() const;

    void setHop(int hop);

    [[nodiscard]] const unsigned char *getParent() const;

    void setParent (const unsigned char *parent);
};

#endif //SCHEDPLUS_NODEIMPL_H
