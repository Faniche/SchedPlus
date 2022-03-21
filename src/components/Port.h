//
// Created by faniche on 2022/1/22.
//

#ifndef SCHEDPLUS_PORT_H
#define SCHEDPLUS_PORT_H


#include <uuid/uuid.h>
#include <vector>

class GateControlEntry {
private:
    std::vector<bool> gateStatesValue;
    u_int64_t timeIntervalValue;
public:
    GateControlEntry();

    [[nodiscard]] const std::vector<bool> &getGateStatesValue() const;

    void setGateStatesValue(int idx, bool gateState);

    [[nodiscard]] u_int64_t getTimeIntervalValue() const;

    void setTimeIntervalValue(u_int64_t timeIntervalValue);
};


class Port {
private:
    uuid_t id{};

    /* the Gigabit link could hold 125000000 bytes per second */
    int speed = 125000000;

    /* the number of available queues for a port */
    int availableQueues = 6;

    /* sending 64 bytes shall spend 512 ns with Gigabit full duplex ethernet */
    int macrotick = 512;

    std::vector<GateControlEntry> gateControlList;



public:
    Port();

    Port(int speed, int availableQueues, int macrotick);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] int getSpeed() const;

    void setSpeed(int speed);

    [[nodiscard]] int getAvailableQueues() const;

    void setAvailableQueues(int availableQueues);

    [[nodiscard]] int getMacrotick() const;

    void setMacrotick(int macrotick);

    [[nodiscard]] const std::vector<GateControlEntry> &getGateControlList() const;

    void addGateControlEntry(const GateControlEntry &gateControlEntry);
};


#endif //SCHEDPLUS_PORT_H
