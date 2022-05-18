//
// Created by faniche on 22-4-19.
//

#ifndef SCHEDPLUS_DELIVERYGUARANTEE_H
#define SCHEDPLUS_DELIVERYGUARANTEE_H

#include <cstdint>

//typedef uint64_t ;
enum DELIVERY_GUARANTEE {
    BANDWIDTH,
    DDL,
    E2E,
    JITTER,
    PKT_LOSE_RATE,
    NONE
};

class DeliveryGuarantee {
private:
    DELIVERY_GUARANTEE type;
    uint64_t lower_val = 0;
    uint64_t upper_val = 0;
    uint64_t lover_obj = 0;
    uint64_t upper_obj = 0;
public:
    DeliveryGuarantee(DELIVERY_GUARANTEE type, uint64_t lowerVal);

    DeliveryGuarantee(DELIVERY_GUARANTEE type, uint64_t lowerVal, uint64_t upperVal);

    [[nodiscard]] DELIVERY_GUARANTEE getType() const;

    [[nodiscard]] uint64_t getLowerVal() const;

    void setLowerVal(uint64_t lowerVal);

    [[nodiscard]] uint64_t getUpperVal() const;

    void setUpperVal(uint64_t upperVal);

    [[nodiscard]] uint64_t getLoverObj() const;

    void setLoverObj(uint64_t loverObj);

    [[nodiscard]] uint64_t getUpperObj() const;

    void setUpperObj(uint64_t upperObj);


};


#endif //SCHEDPLUS_DELIVERYGUARANTEE_H
