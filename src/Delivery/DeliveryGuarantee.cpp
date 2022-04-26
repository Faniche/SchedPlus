//
// Created by faniche on 22-4-19.
//

#include "DeliveryGuarantee.h"

DeliveryGuarantee::DeliveryGuarantee(DELIVERY_GUARANTEE type, uint32_t lowerVal) : type(type), lower_val(lowerVal) {}

DeliveryGuarantee::DeliveryGuarantee(DELIVERY_GUARANTEE type, uint32_t lowerVal, uint32_t upperVal) : type(type),
                                                                                            lower_val(lowerVal),
                                                                                            upper_val(upperVal) {}

DELIVERY_GUARANTEE DeliveryGuarantee::getType() const {
    return type;
}

uint32_t DeliveryGuarantee::getLowerVal() const {
    return lower_val;
}

void DeliveryGuarantee::setLowerVal(uint32_t lowerVal) {
    lower_val = lowerVal;
}

uint32_t DeliveryGuarantee::getUpperVal() const {
    return upper_val;
}

void DeliveryGuarantee::setUpperVal(uint32_t upperVal) {
    upper_val = upperVal;
}

uint32_t DeliveryGuarantee::getLoverObj() const {
    return lover_obj;
}

void DeliveryGuarantee::setLoverObj(uint32_t loverObj) {
    lover_obj = loverObj;
}

uint32_t DeliveryGuarantee::getUpperObj() const {
    return upper_obj;
}

void DeliveryGuarantee::setUpperObj(uint32_t upperObj) {
    upper_obj = upperObj;
}
