//
// Created by faniche on 2022/1/22.
//

#include "Port.h"

GateControlEntry::GateControlEntry() {
    std::vector<bool> tmp(8, false);
    gateStatesValue = tmp;
    timeIntervalValue = 0;
}


const std::vector<bool> &GateControlEntry::getGateStatesValue() const {
    return gateStatesValue;
}

void GateControlEntry::setGateStatesValue(int idx, bool gateState) {
    gateStatesValue.at(gateState);
}

u_int64_t GateControlEntry::getTimeIntervalValue() const {
    return timeIntervalValue;
}

void GateControlEntry::setTimeIntervalValue(u_int64_t _timeIntervalValue) {
    GateControlEntry::timeIntervalValue = _timeIntervalValue;
}

Port::Port() {
    uuid_generate(this->id);
}

Port::Port(int speed, int availableQueues, int macrotick) : speed(speed), availableQueues(availableQueues),
                                                            macrotick(macrotick) {
    uuid_generate(this->id);
}


const unsigned char *Port::getId() const {
    return id;
}

int Port::getSpeed() const {
    return speed;
}

void Port::setSpeed(int _speed) {
    this->speed = _speed;
}

int Port::getAvailableQueues() const {
    return availableQueues;
}

void Port::setAvailableQueues(int _availableQueues) {
    this->availableQueues = _availableQueues;
}

int Port::getMacrotick() const {
    return macrotick;
}

void Port::setMacrotick(int _macrotick) {
    this->macrotick = _macrotick;
}

const std::vector<GateControlEntry> &Port::getGateControlList() const {
    return gateControlList;
}

void Port::addGateControlEntry(const GateControlEntry &gateControlEntry) {
    gateControlList.push_back(gateControlEntry);
}
