/**
 * File: Components.h
 * Date: 2021/11/28
 * Author: Faniche
 */

#ifndef SCHEDPLUS_COMPONENTS_H
#define SCHEDPLUS_COMPONENTS_H

#include <uuid/uuid.h>
#include <map>

enum {
    SWITCH,
    END_SYSTEM
};

class Node {
private:
    /* identify of an end system */
    uuid_t id{};

    /* the name of an end system */
    std::string name{};

    /* sending 64 bytes shall spend 512 ns with Gigabit full duplex ethernet */
    int macrotick = 512;

public:
    explicit Node(const std::string &_name) : name(_name) {
        uuid_generate(this->id);
        this->name = _name;
    };

    Node(const std::string &_name, int _macrotick) : name(_name), macrotick(_macrotick) {
        uuid_generate(this->id);
        this->name = _name;
        this->macrotick = _macrotick;
    };

    [[nodiscard]] const unsigned char *getId() const {
        return id;
    }

    [[nodiscard]] const std::string &getName() const {
        return name;
    }

    void setName(const std::string &_name) {
        Node::name = _name;
    }

    [[nodiscard]] int getMacrotick() const {
        return macrotick;
    }

    void setMacrotick(int _macrotick) {
        Node::macrotick = _macrotick;
    }
};

class Port {
private:
    uuid_t id{};

    /* the Gigabit link could hold 125000000 bytes per second */
    int speed = 125000000;

    /* the number of available queues for a port */
    int availableQueues = 6;

public:
    Port() {
        uuid_generate(this->id);
    }

    Port(int _speed, int _availableQueues) : speed(_speed), availableQueues(_availableQueues) {
        this->speed = _speed;
        this->availableQueues = _availableQueues;
    }

    [[nodiscard]] const unsigned char *getId() const {
        return id;
    }

    [[nodiscard]] int getSpeed() const {
        return speed;
    }

    void setSpeed(int _speed) {
        Port::speed = _speed;
    }

    [[nodiscard]] int getAvailableQueues() const {
        return availableQueues;
    }

    void setAvailableQueues(int _availableQueues) {
        Port::availableQueues = _availableQueues;
    }
};

class EndSystem : public Node {
private:
    Port port;
    Port connectedSwitch;
    int nodeType = END_SYSTEM;
public:
    explicit EndSystem(const std::string &_name) : Node(_name) {
        Port _port;
        this->port = _port;
    }

    EndSystem(const std::string &_name, int _macortick, int _speed, int _availableQueues) : Node(_name, _macortick) {
        Port _port(_speed, _availableQueues);
        this->port = _port;
    }

    [[nodiscard]] const Port &getPort() const {
        return port;
    }

    void setPort(const Port &_port) {
        EndSystem::port = _port;
    }

    [[nodiscard]] const Port &getConnectedSwitch() const {
        return connectedSwitch;
    }

    void setConnectedSwitch(Port switchPort) {
        this->connectedSwitch = switchPort;
    }

    void setSpeed(int _speed) {
        this->port.setSpeed(_speed);
    }

    void setAvailableQueues(int _availableQueues) {
        this->port.setAvailableQueues(_availableQueues);
    }

    [[nodiscard]] int getNodeType() const {
        return nodeType;
    }
};

class Switch : public Node {
private:
    /* the number of ports of a switch */
    int portNum = 8;

    // std::vector<SwPort> ports;
    std::vector<Port> ports;

    std::map<Port, Port> portToES;

    int nodeType = SWITCH;

    void initPorts() {
        for (int i = 0; i < this->portNum; ++i) {
            Port port;
            this->ports.push_back(port);
        }
    }

public:
    explicit Switch(const std::string &_name) : Node(_name) {
        initPorts();
    }

    Switch(const std::string &_name, int _macrotick, int _portNum) : Node(_name, _macrotick) {
        this->portNum = _portNum;
        initPorts();
    }

    [[nodiscard]] int getPortNum() const {
        return portNum;
    }

    void setPortNum(int _portNum) {
        Switch::portNum = _portNum;
    }

    [[nodiscard]] const std::vector<Port> &getPorts() const {
        return ports;
    }

    void setPortSpeed(int idx, int _speed) {
        this->ports.at(idx).setSpeed(_speed);
    }

    void setPortAvailableQueues(int idx, int _availableQueues) {
        this->ports.at(idx).setAvailableQueues(_availableQueues);
    }

    [[nodiscard]] int getNodeType() const {
        return nodeType;
    }

    void setConnectedES(const EndSystem &endSystem, int portIdx) {
        this->portToES[ports[portIdx]] = endSystem.getPort();
    }

    void setConnectedSW(const Port &srcPort, const Port &destPort) {
        this->portToES[srcPort] = destPort;
    }
};

class Flow {
private:
    uuid_t id{};

    /* The sending offset in a hyperperiod of each frame */
    int offset{};

    /* The interval between two frames of a Flow */
    int period{};

    /* The frame length include network headers of a packet of each frame */
    int frameLength{};

    /* The source node of a Flow */
    EndSystem src;

    /* The destination node of a Flow */
    EndSystem dest;

    /* the Flow is time-sensitive or not */
    bool isCritical = true;

    /* the Replication of frame */
    int rep = 1;

    /* the Flow is multicast or not */
    bool multicast = false;

    std::vector<std::vector<Port>> routes;

public:
    Flow(const EndSystem &_src, const EndSystem &_dest) : src(_src),
                                                          dest(_dest) {
        uuid_generate(this->id);
        this->src = _src;
        this->dest = _dest;
    }

    Flow(int _offset, int _period, int _frameLength, const EndSystem &_src, const EndSystem &_dest) : offset(_offset),
                                                                                                      period(_period),
                                                                                                      frameLength(
                                                                                                              _frameLength),
                                                                                                      src(_src),
                                                                                                      dest(_dest) {
        uuid_generate(this->id);
        this->offset = _offset;
        this->period = _period;
        this->frameLength = _frameLength;
        this->src = _src;
        this->dest = _dest;
    }

    [[nodiscard]] const unsigned char *getId() const {
        return id;
    }


    [[nodiscard]] int getOffset() const {
        return offset;
    }

    void setOffset(int _offset) {
        Flow::offset = _offset;
    }

    [[nodiscard]] int getPeriod() const {
        return period;
    }

    void setPeriod(int _period) {
        Flow::period = _period;
    }

    [[nodiscard]] int getFrameLength() const {
        return frameLength;
    }

    void setFrameLength(int _frameLength) {
        Flow::frameLength = _frameLength;
    }

    [[nodiscard]] const EndSystem &getSrc() const {
        return src;
    }

    void setSrc(const EndSystem &_src) {
        Flow::src = _src;
    }

    [[nodiscard]] const EndSystem &getDest() const {
        return dest;
    }

    void setDest(const EndSystem &_dest) {
        Flow::dest = _dest;
    }

    [[nodiscard]] bool isCritical1() const {
        return isCritical;
    }

    void setIsCritical(bool _isCritical) {
        Flow::isCritical = _isCritical;
    }

    [[nodiscard]] int getRep() const {
        return rep;
    }

    void setRep(int _rep) {
        Flow::rep = _rep;
    }


    [[nodiscard]] bool isMulticast() const {
        return multicast;
    }

    void setMulticast(bool _multicast) {
        Flow::multicast = _multicast;
    }
};


#endif //SCHEDPLUS_COMPONENTS_H
