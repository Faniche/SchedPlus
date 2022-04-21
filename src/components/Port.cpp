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

void GateControlEntry::setGateStatesValue(int idx, GATE_EVENT gateState) {
    gateStatesValue.at(idx) = gateState;
}

u_int64_t GateControlEntry::getStartTime() const {
    return startTime;
}

void GateControlEntry::setStartTime(u_int64_t _startTime) {
    GateControlEntry::startTime = _startTime;
}

u_int64_t GateControlEntry::getTimeIntervalValue() const {
    return timeIntervalValue;
}

void GateControlEntry::setTimeIntervalValue(u_int64_t _timeIntervalValue) {
    GateControlEntry::timeIntervalValue = _timeIntervalValue;
}

/* Impl of Port*/

Port::Port() {
    uuid_generate(this->id);
    frameQueue = std::vector(8, 0);
}

Port::Port(int _speed) : speed(_speed) {
    uuid_generate(this->id);
    macrotick = (_speed / 10 ^ 9);
    frameQueue = std::vector(8, 0);
}


const unsigned char *Port::getId() const {
    return id;
}

int Port::getSpeed() const {
    return speed;
}

int Port::getMacrotick() const {
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

int Port::getQsize() const {
    return qsize;
}

void Port::setQsize(int _qsize) {
    Port::qsize = _qsize;
}

int Port::getQlen() const {
    return qlen;
}

void Port::setQlen(int _qlen) {
    Port::qlen = _qlen;
}

const std::vector<int> &Port::getFrameQueue() const {
    return frameQueue;
}

void Port::setFrameQueue(const std::vector<int> &_frameQueue) {
    Port::frameQueue = _frameQueue;
}
