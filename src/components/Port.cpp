//
// Created by faniche on 2022/1/22.
//

//#include <spdlog/spdlog.h>
#include "../../lib/spdlog/include/spdlog/spdlog.h"
#include "Port.h"
#include "../include/type.h"

GateControlEntry::GateControlEntry() {
    gateStatesValue.assign(8, GATE_OPEN);
    timeIntervalValue = 0;
}


const std::vector<bool> &GateControlEntry::getGateStatesValue() const {
    return gateStatesValue;
}

void GateControlEntry::setGateStatesValue(uint8_t idx, GATE_EVENT gateState) {
    gateStatesValue[idx] = gateState;
}

uint64_t GateControlEntry::getStartTime() const {
    return startTime;
}

void GateControlEntry::setStartTime(uint64_t _startTime) {
    GateControlEntry::startTime = _startTime;
}

uint64_t GateControlEntry::getTimeIntervalValue() const {
    return timeIntervalValue;
}

void GateControlEntry::setTimeIntervalValue(uint64_t _timeIntervalValue) {
    GateControlEntry::timeIntervalValue = _timeIntervalValue;
}

std::string GateControlEntry::toBitVec() const {
    std::string bitvector_str;
    for (auto const &gate_event: gateStatesValue) {
        if (gate_event == GATE_OPEN)
            bitvector_str.append("1");
        else
            bitvector_str.append("0");
    }
    std::reverse(bitvector_str.begin(), bitvector_str.end());
    return bitvector_str;
}

/* Impl of Port*/

Port::Port() {
    uuid_generate(this->id);
    frameQueue.assign(8, 0);
}

Port::Port(uint64_t _speed) : speed(_speed) {
    uuid_generate(this->id);
    macrotick = (_speed / 10 ^ 9);
    frameQueue.assign(8, 0);
}


const unsigned char *Port::getId() const {
    return id;
}

uint64_t Port::getSpeed() const {
    return speed;
}

uint8_t Port::getMacrotick() const {
    return macrotick;
}

const std::vector<GateControlEntry> &Port::getGateControlList() const {
    return gateControlList;
}

void Port::addGateControlEntry(const GateControlEntry &gateControlEntry) {
    gateControlList.push_back(gateControlEntry);
}

void Port::clearGCL() {
    gateControlList.clear();
    gateControlList.shrink_to_fit();
}

bool Port::compareGCL(const GateControlEntry & a, const GateControlEntry &b){
    return a.getStartTime() < b.getStartTime();
}

void Port::sortGCL() {
    std::sort(gateControlList.begin(), gateControlList.end(), compareGCL);
}

void Port::mergeGCL() {
    spdlog::get("console")->set_level(spdlog::level::debug);
    for (int i = 0; (i + 1) < gateControlList.size(); ++i) {
        if (gateControlList[i].getStartTime() + gateControlList[i].getTimeIntervalValue() + schedplus::IFG_TIME >= gateControlList[i + 1].getStartTime()) {
            spdlog::get("console")->debug("before merge");
            spdlog::get("console")->debug("cur.start = {}, cur.finish = {}, IFG.finish = {}",
                                          gateControlList[i].getStartTime(),
                                          gateControlList[i].getStartTime() + gateControlList[i].getTimeIntervalValue(),
                                          gateControlList[i].getStartTime() + gateControlList[i].getTimeIntervalValue() + schedplus::IFG_TIME);
            spdlog::get("console")->debug("nxt.start = {}, nxt.finish = {}",
                                          gateControlList[i + 1].getStartTime(), gateControlList[i + 1].getStartTime() + gateControlList[i + 1].getTimeIntervalValue());
            uint64_t queue_delay = gateControlList[i].getStartTime() + gateControlList[i].getTimeIntervalValue() - gateControlList[i + 1].getStartTime();
            uint64_t finish_time = gateControlList[i + 1].getStartTime() + gateControlList[i + 1].getTimeIntervalValue() + queue_delay;
            uint64_t time_interval = finish_time - gateControlList[i].getStartTime() + schedplus::IFG_TIME;
            gateControlList[i].setTimeIntervalValue(time_interval);
            gateControlList[i].setGateStatesValue(5, GATE_OPEN);
            gateControlList[i].setGateStatesValue(6, GATE_OPEN);
            gateControlList.erase(gateControlList.begin() + (i + 1));
        }
    }
}

bool Port::checkGCLCollision() {
    for (int i = 0; i < gateControlList.size() - 1; ++i) {
        if (gateControlList[i].getStartTime() + gateControlList[i].getTimeIntervalValue() > gateControlList[i + 1].getStartTime())
            return false;
    }
    return true;
}

uint64_t Port::getQsize() const {
    return qsize;
}

void Port::setQsize(uint64_t _qsize) {
    Port::qsize = _qsize;
}

uint16_t Port::getQlen() const {
    return qlen;
}

void Port::setQlen(uint16_t _qlen) {
    Port::qlen = _qlen;
}

const std::vector<uint64_t> &Port::getFrameQueue() const {
    return frameQueue;
}

void Port::setFrameQueue(const std::vector<uint64_t> &_frameQueue) {
    Port::frameQueue = _frameQueue;
}
