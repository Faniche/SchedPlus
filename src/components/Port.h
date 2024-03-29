//
// Created by faniche on 2022/1/22.
//

#ifndef SCHEDPLUS_PORT_H
#define SCHEDPLUS_PORT_H
//
//#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
//#define SPDLOG_TRACE_ON
//#define SPDLOG_DEBUG_ON

#include <uuid/uuid.h>
#include <vector>
#include <cstdint>
#include <string>
//#include <mutex>

typedef bool GATE_EVENT;
#define GATE_CLOSE  false
#define GATE_OPEN   true

class GateControlEntry {
private:
    std::vector<GATE_EVENT> gateStatesValue;
//    std::atomic<std::vector<GATE_EVENT>> gateStatesValue;
    uint64_t startTime;
    uint64_t timeIntervalValue;

public:
    GateControlEntry();

    [[nodiscard]] const std::vector<bool> &getGateStatesValue() const;

    void setGateStatesValue(uint8_t idx, GATE_EVENT gateState);

    [[nodiscard]] uint64_t getStartTime() const;

    void setStartTime(uint64_t startTime);

    [[nodiscard]] uint64_t getTimeIntervalValue() const;

    void setTimeIntervalValue(uint64_t timeIntervalValue);

    std::string toBitVec() const;
};


class Port {
private:
    uuid_t id{};

    /* speed of port, default: 1Gbps */
    uint64_t speed = 1000000000;

    /* sending 1 byte shall spend 8 ns with Gigabit full duplex ethernet */
    uint8_t macrotick = 8;

    std::vector<GateControlEntry> gateControlList;

    /* the bytes of cached frame in queue with priority 5 and 6 */
    uint64_t qsize = 0;

    /* the num of cached frame in queue with priority 5 and 6 */
    uint16_t qlen = 0;

    std::vector<uint64_t> frameQueue;

public:
    Port();

    explicit Port(uint64_t speed);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] uint64_t getSpeed() const;

    [[nodiscard]] uint8_t getMacrotick() const;

    [[nodiscard]] const std::vector<GateControlEntry> &getGateControlList() const;

    void addGateControlEntry(const GateControlEntry &gateControlEntry);

    void clearGCL();

    static bool compareGCL(const GateControlEntry & a, const GateControlEntry &b);

    void sortGCL();

    void mergeGCL();

    bool checkGCLCollision();

    [[nodiscard]] uint64_t getQsize() const;

    void setQsize(uint64_t qsize);

    [[nodiscard]] uint16_t getQlen() const;

    void setQlen(uint16_t qlen);

    [[nodiscard]] const std::vector<uint64_t> &getFrameQueue() const;

    void setFrameQueue(const std::vector<uint64_t> &frameQueue);

};


#endif //SCHEDPLUS_PORT_H
