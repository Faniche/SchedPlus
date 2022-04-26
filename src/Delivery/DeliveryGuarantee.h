//
// Created by faniche on 22-4-19.
//

#ifndef SCHEDPLUS_DELIVERYGUARANTEE_H
#define SCHEDPLUS_DELIVERYGUARANTEE_H

#include <cstdint>

//typedef uint32_t ;
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
    uint32_t lower_val = 0;
    uint32_t upper_val = 0;
    uint32_t lover_obj = 0;
    uint32_t upper_obj = 0;
public:
    DeliveryGuarantee(DELIVERY_GUARANTEE type, uint32_t lowerVal);

    DeliveryGuarantee(DELIVERY_GUARANTEE type, uint32_t lowerVal, uint32_t upperVal);

    [[nodiscard]] DELIVERY_GUARANTEE getType() const;

    [[nodiscard]] uint32_t getLowerVal() const;

    void setLowerVal(uint32_t lowerVal);

    [[nodiscard]] uint32_t getUpperVal() const;

    void setUpperVal(uint32_t upperVal);

    [[nodiscard]] uint32_t getLoverObj() const;

    void setLoverObj(uint32_t loverObj);

    [[nodiscard]] uint32_t getUpperObj() const;

    void setUpperObj(uint32_t upperObj);


};


#endif //SCHEDPLUS_DELIVERYGUARANTEE_H
