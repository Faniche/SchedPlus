//
// Created by faniche on 22-4-19.
//

#include "DeliveryGuarantee.h"

DeliveryGuarantee::DeliveryGuarantee(DELIVERY_GUARANTEE type, int lowerVal) : type(type), lower_val(lowerVal) {}

DeliveryGuarantee::DeliveryGuarantee(DELIVERY_GUARANTEE type, int lowerVal, int upperVal) : type(type),
                                                                                            lower_val(lowerVal),
                                                                                            upper_val(upperVal) {}

DELIVERY_GUARANTEE DeliveryGuarantee::getType() const {
    return type;
}

int DeliveryGuarantee::getLowerVal() const {
    return lower_val;
}

void DeliveryGuarantee::setLowerVal(int lowerVal) {
    lower_val = lowerVal;
}

int DeliveryGuarantee::getUpperVal() const {
    return upper_val;
}

void DeliveryGuarantee::setUpperVal(int upperVal) {
    upper_val = upperVal;
}

int DeliveryGuarantee::getLoverObj() const {
    return lover_obj;
}

void DeliveryGuarantee::setLoverObj(int loverObj) {
    lover_obj = loverObj;
}

int DeliveryGuarantee::getUpperObj() const {
    return upper_obj;
}

void DeliveryGuarantee::setUpperObj(int upperObj) {
    upper_obj = upperObj;
}
