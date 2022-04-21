//
// Created by faniche on 22-4-19.
//

#ifndef SCHEDPLUS_DELIVERYGUARANTEE_H
#define SCHEDPLUS_DELIVERYGUARANTEE_H

//typedef int ;
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
    int lower_val = 0;
    int upper_val = 0;
    int lover_obj = 0;
    int upper_obj = 0;
public:
    DeliveryGuarantee(DELIVERY_GUARANTEE type, int lowerVal);

    DeliveryGuarantee(DELIVERY_GUARANTEE type, int lowerVal, int upperVal);

    [[nodiscard]] DELIVERY_GUARANTEE getType() const;

    [[nodiscard]] int getLowerVal() const;

    void setLowerVal(int lowerVal);

    [[nodiscard]] int getUpperVal() const;

    void setUpperVal(int upperVal);

    [[nodiscard]] int getLoverObj() const;

    void setLoverObj(int loverObj);

    [[nodiscard]] int getUpperObj() const;

    void setUpperObj(int upperObj);


};


#endif //SCHEDPLUS_DELIVERYGUARANTEE_H
