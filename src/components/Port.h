//
// Created by faniche on 2022/1/22.
//

#ifndef SCHEDPLUS_PORT_H
#define SCHEDPLUS_PORT_H


#include <uuid/uuid.h>
#include <vector>

typedef bool GATE_EVENT;
#define GATE_CLOSE  false
#define GATE_OPEN   true

class GateControlEntry {
private:
    std::vector<GATE_EVENT> gateStatesValue;
    u_int64_t startTime;
    u_int64_t timeIntervalValue;

public:
    GateControlEntry();

    [[nodiscard]] const std::vector<bool> &getGateStatesValue() const;

    void setGateStatesValue(int idx, GATE_EVENT gateState);

    u_int64_t getStartTime() const;

    void setStartTime(u_int64_t startTime);

    [[nodiscard]] u_int64_t getTimeIntervalValue() const;

    void setTimeIntervalValue(u_int64_t timeIntervalValue);
};


class Port {
private:
    uuid_t id{};

    /* speed of port, default: 1Gbps */
    int speed = 1000000000;

    /* sending 1 byte shall spend 8 ns with Gigabit full duplex ethernet */
    int macrotick = 8;

    std::vector<GateControlEntry> gateControlList;

    /* the bytes of cached frame in queue with priority 5 and 6 */
    int qsize = 0;

    /* the num of cached frame in queue with priority 5 and 6 */
    int qlen = 0;

    std::vector<int> frameQueue;

public:
    Port();

    explicit Port(int speed);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] int getSpeed() const;

    [[nodiscard]] int getMacrotick() const;

    [[nodiscard]] const std::vector<GateControlEntry> &getGateControlList() const;

    void addGateControlEntry(const GateControlEntry &gateControlEntry);

    [[nodiscard]] int getQsize() const;

    void setQsize(int qsize);

    [[nodiscard]] int getQlen() const;

    void setQlen(int qlen);

    [[nodiscard]] const std::vector<int> &getFrameQueue() const;

    void setFrameQueue(const std::vector<int> &frameQueue);
};


#endif //SCHEDPLUS_PORT_H
