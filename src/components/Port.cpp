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
    gateStatesValue.at(gateState);
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

void Port::addGateControlEntry(const GateControlEntry &gateControlEntry) {
    gateControlList.push_back(gateControlEntry);
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
