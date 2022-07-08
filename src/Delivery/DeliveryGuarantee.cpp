//
// Created by faniche on 22-4-19.
//

#include <iostream>
#include "DeliveryGuarantee.h"

DeliveryGuarantee::DeliveryGuarantee(DELIVERY_GUARANTEE type, uint64_t lowerVal) : type(type), lower_val(lowerVal) {}

DeliveryGuarantee::DeliveryGuarantee(DELIVERY_GUARANTEE type, uint64_t lowerVal, uint64_t upperVal) : type(type),
                                                                                            lower_val(lowerVal),
                                                                                            upper_val(upperVal) {}

DELIVERY_GUARANTEE DeliveryGuarantee::getType() const {
    return type;
}

uint64_t DeliveryGuarantee::getLowerVal() const {
    return lower_val;
}

void DeliveryGuarantee::setLowerVal(uint64_t lowerVal) {
    lower_val = lowerVal;
}

uint64_t DeliveryGuarantee::getUpperVal() const {
    return upper_val;
}

void DeliveryGuarantee::setUpperVal(uint64_t upperVal) {
    upper_val = upperVal;
}

uint64_t DeliveryGuarantee::getLoverObj() const {
    return lover_obj;
}

void DeliveryGuarantee::setLoverObj(uint64_t loverObj) {
    lover_obj = loverObj;
}

uint64_t DeliveryGuarantee::getUpperObj() const {
    return upper_obj;
}

void DeliveryGuarantee::setUpperObj(uint64_t upperObj) {
    upper_obj = upperObj;
}

std::string DeliveryGuarantee::toString(const DeliveryGuarantee& deliveryGuarantee) {
    std::string ret;
    switch (deliveryGuarantee.getType()) {
        case BANDWIDTH:
            ret.append("BANDWIDTH: { ");
            ret.append("lover: " + std::to_string(deliveryGuarantee.getLowerVal()) + ", ");
            ret.append("upper: " + std::to_string(deliveryGuarantee.getUpperVal()) + "}");
            break;
        case DDL:
            ret.append("DDL: { ");
            ret.append("lover: " + std::to_string(deliveryGuarantee.getLowerVal()) + "}");
            break;
        case E2E:
            ret.append("E2E: { ");
            ret.append("lover: " + std::to_string(deliveryGuarantee.getLowerVal()) + "}");
            break;
        case JITTER:
            ret.append("JITTER: { ");
            ret.append("lover: " + std::to_string(deliveryGuarantee.getLowerVal()) + ", ");
            ret.append("upper: " + std::to_string(deliveryGuarantee.getUpperVal()) + "}");
            break;
        case PKT_LOSE_RATE:
            ret.append("PKT_LOSE_RATE: { ");
            ret.append("lover: " + std::to_string(deliveryGuarantee.getLowerVal()) + ", ");
            ret.append("upper: " + std::to_string(deliveryGuarantee.getUpperVal()) + "}");
            break;
        case NONE:
            ret.append("NONE");
            break;
        default:
            std::cerr << "Some error";
    }
    return ret;
}
