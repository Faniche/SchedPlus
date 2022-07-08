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

#endif //SCHEDPLUS_NODEIMPL_H
