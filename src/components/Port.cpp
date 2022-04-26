//
// Created by faniche on 2022/1/22.
//

#include "Port.h"

GateControlEntry::GateControlEntry() {
    std::vector<bool> tmp(8, GATE_CLOSE);
    gateStatesValue = tmp;
    timeIntervalValue = 0;
}


const std::vector<bool> &GateControlEntry::getGateStatesValue() const {
    return gateStatesValue;
}

void GateControlEntry::setGateStatesValue(uint8_t idx, GATE_EVENT gateState) {
    gateStatesValue[idx] = gateState;
}

uint32_t GateControlEntry::getStartTime() const {
    return startTime;
}

void GateControlEntry::setStartTime(uint32_t _startTime) {
    GateControlEntry::startTime = _startTime;
}

uint32_t GateControlEntry::getTimeIntervalValue() const {
    return timeIntervalValue;
}

void GateControlEntry::setTimeIntervalValue(uint32_t _timeIntervalValue) {
    GateControlEntry::timeIntervalValue = _timeIntervalValue;
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

void Port::addGateControlEntry(const GateControlEntry &gateControlEntry, std::mutex &gcl_lock) {
    gcl_lock.lock();
    gateControlList.push_back(gateControlEntry);
    gcl_lock.unlock();
}

bool Port::compareGCL(const GateControlEntry & a, const GateControlEntry &b){
    return a.getStartTime() < b.getStartTime();
}

void Port::sortGCL(std::mutex &gcl_lock) {
    gcl_lock.lock();
    std::sort(gateControlList.begin(), gateControlList.end(), compareGCL);
    gcl_lock.unlock();
}

bool Port::checkGCLCollision() {
    for (int i = 0; i < gateControlList.size() - 1; ++i) {
        if (gateControlList[i].getStartTime() + gateControlList[i].getTimeIntervalValue() > gateControlList[i + 1].getStartTime())
            return false;
    }
    return true;
}

uint32_t Port::getQsize() const {
    return qsize;
}

void Port::setQsize(uint32_t _qsize) {
    Port::qsize = _qsize;
}

uint16_t Port::getQlen() const {
    return qlen;
}

void Port::setQlen(uint16_t _qlen) {
    Port::qlen = _qlen;
}

const std::vector<uint32_t> &Port::getFrameQueue() const {
    return frameQueue;
}

void Port::setFrameQueue(const std::vector<uint32_t> &_frameQueue) {
    Port::frameQueue = _frameQueue;
}
