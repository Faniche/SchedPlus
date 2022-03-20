//
// Created by faniche on 2022/1/25.
//

#include "Flow.h"
#include "Link.h"

#include <sstream>

Flow::Flow(int period, int deadline, int frameLength, Node *src, Node *dest, bool isCritical, int rep, bool multicast)
        : period(period),
        deadline(deadline),
        frameLength(frameLength),
        src(src),
        dest(dest),
        isCritical(isCritical),
        rep(rep),
        multicast(multicast) {
    uuid_generate(id);
    priorityCodePoint = BE;
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

PRIORITY_CODE_POINT Flow::getPriorityCodePoint() const {
    return priorityCodePoint;
}

void Flow::setPriorityCodePoint(PRIORITY_CODE_POINT _priorityCodePoint) {
    Flow::priorityCodePoint = _priorityCodePoint;
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

const std::vector<std::vector<DirectedLink *>> &Flow::getRoutes() const {
    return routes;
}

void Flow::setRoutes(const std::vector<DirectedLink *> &route) {
    routes.push_back(route);
}

std::string Flow::toString(std::ostringstream &oss) {
    oss << "{\n";
    oss << "\t" << R"("id": ")" << id << "\"," << std::endl;
    oss << "\t" << R"("offset": )" << offset << "," << std::endl;
    oss << "\t" << R"("period": )" << period << "," << std::endl;
    oss << "\t" << R"("deadline": )" << deadline << "," << std::endl;
    oss << "\t" << R"("srcNode": ")" << src->getName() << "," << std::endl;
    oss << "\t" << R"("destNode": ")" << dest->getName() << "," << std::endl;
    oss << "\t" << R"("isCritical": )" << std::boolalpha << isCritical << "," << std::endl;
    oss << "\t" << R"("rep": )" << rep << "," << std::endl;
    oss << "\t" << R"("multicast": )" << std::boolalpha << multicast;
    if (routes.empty()) {
        oss << std::endl;
    } else {
        oss << "," << std::endl;
        oss << "\t" << R"("routes": {)";
        for (size_t i = 0; i < routes.size(); ++i) {
            oss << std::endl << "\t\t\"route" << i << "\": \"" << routes[i][0]->getSrcNode()->getName();
            for (auto & j : routes[i]) {
                oss << " -> " << j->getDestNode()->getName();
            }
            oss << "\",";
        }
        oss.seekp(-1, std::ios_base::end);
        oss << std::endl << "\t" << "}" << std::endl;
    }
    oss << "}";
    return oss.str();
}