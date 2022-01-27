//
// Created by faniche on 2022/1/24.
//

#ifndef SCHEDPLUS_LINK_H
#define SCHEDPLUS_LINK_H


#include <uuid/uuid.h>
#include "Port.h"

class Link {
private:
    uuid_t id{};
    Port port_a;
    Port port_b;
public:
    Link(const Port &portA, const Port &portB);

    [[nodiscard]] const unsigned char *getId() const;

    [[nodiscard]] const Port &getPortA() const;

    [[nodiscard]] const Port &getPortB() const;
};


#endif //SCHEDPLUS_LINK_H
