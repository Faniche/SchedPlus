//
// Created by faniche on 2022/1/25.
//

#include "Flow.h"
#include "Link.h"

#include <sstream>

Flow::Flow(int period, int deadline, int frameLength, Node *src, Node *dest, bool isCritical, int rep, bool multicast) : period(
        period), deadline(deadline), frameLength(frameLength), src(src), dest(dest), isCritical(isCritical), rep(rep), multicast(
        multicast) {
    uuid_generate(id);
}

const unsigned char *Flow::getId() const {
    return id;
}

int Flow::getOffset() const {
    return offset;
}

void Flow::setOffset(int _offset) {
    Flow::offset = _offset;
}

int Flow::getPeriod() const {
    return period;
}

int Flow::getDeadline() const {
    return deadline;
}

int Flow::getFrameLength() const {
    return frameLength;
}

Node *Flow::getSrc() const {
    return src;
}

Node *Flow::getDest() const {
    return dest;
}

bool Flow::isCritical1() const {
    return isCritical;
}

int Flow::getRep() const {
    return rep;
}

bool Flow::isMulticast() const {
    return multicast;
}

std::string Flow::toString(std::ostringstream &oss) {
    oss << "{\n";
    oss << "\t" << R"("id": ")" << id << "\"," << std::endl;
    oss << "\t" << R"("offset": )" << offset << "," << std::endl;
    oss << "\t" << R"("period": )" << period << "," << std::endl;
    oss << "\t" << R"("deadline": )" << deadline << "," << std::endl;
    oss << "\t" << R"("src": ")" << src->getName() << "," << std::endl;
    oss << "\t" << R"("dest": ")" << dest->getName() << "," << std::endl;
    oss << "\t" << R"("isCritical": )" << std::boolalpha << isCritical << "," << std::endl;
    oss << "\t" << R"("rep": )" << rep << "," << std::endl;
    oss << "\t" << R"("multicast": )" << std::boolalpha << multicast;
    if (routes.empty()) {
        oss << std::endl;
    } else {
        oss << "," << std::endl;
        oss << "\t" << R"("routes": [)" << std::endl;
//        for (size_t i = 0; i < routes.size(); ++i) {
//            oss << "\t\t\"route" << i << "\": [" << std::endl;
//            for (Link link: routes.at(i)) {
//                oss << "\t\t\t{" <<
//            }
//        }
//        oss << "\t\t" <<  << std::endl;
        oss << "\t" << "]" << std::endl;
    }
    oss << "}" << std::endl;
    return oss.str();
}