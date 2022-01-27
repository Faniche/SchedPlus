//
// Created by faniche on 2022/1/24.
//

#include "Link.h"

Link::Link(const Port &portA, const Port &portB) : port_a(portA), port_b(portB) {
    uuid_generate(id);
}

const unsigned char *Link::getId() const {
    return id;
}

const Port &Link::getPortA() const {
    return port_a;
}

const Port &Link::getPortB() const {
    return port_b;
}