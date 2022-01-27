//
// Created by faniche on 2022/1/25.
//

#include "Flow.h"

#include <utility>
#include <sstream>

Flow::Flow(int period, int frameLength, Node *src, Node *dest, bool isCritical, int rep, bool multicast) : period(
        period), frameLength(frameLength), src(src), dest(dest), isCritical(isCritical), rep(rep), multicast(
        multicast) {
    uuid_generate(id);
}

std::string Flow::toString(std::ostringstream &oss) {
    oss << "{\n";
    oss << "\t" << R"("id": ")" << id << "\"," << std::endl;
    oss << "\t" << R"("offset": )" << offset << "," << std::endl;
    oss << "\t" << R"("period": )" << period << "," << std::endl;
    oss << "\t" << R"("src": ")" << src->getName() << "," << std::endl;
    oss << "\t" << R"("dest": ")" << dest->getName() << "," << std::endl;
    oss << "\t" << R"("isCritical": )" << std::boolalpha << isCritical << "," << std::endl;
    oss << "\t" << R"("rep": )" << rep << "," << std::endl;
    oss << "\t" << R"("multicast": )" << std::boolalpha << multicast << "," << std::endl;
    oss << "\t" << R"("routes": )" << "routes" << std::endl;
    oss << "}" << std::endl;
//    ret.append(R"("routes": ")");       ret.append((char *)id);     ret.append(",\n\t");
    return oss.str();
}
