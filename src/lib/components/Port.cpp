//
// Created by faniche on 2022/1/22.
//

#include "Port.h"

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

void Port::setSpeed(int speed) {
    this->speed = speed;
}

int Port::getAvailableQueues() const {
    return availableQueues;
}

void Port::setAvailableQueues(int availableQueues) {
    this->availableQueues = availableQueues;
}

int Port::getMacrotick() const {
    return macrotick;
}

void Port::setMacrotick(int macrotick) {
    this->macrotick = macrotick;
}

