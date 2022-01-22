#include <iostream>
#include <vector>
#include "src/Components.h"
#include "src/Utils.h"

using std::cout;    using std::endl;
using std::vector;

int main() {
    EndSystem e1("e1");
    EndSystem e2("e2");
    EndSystem e3("e3");
    EndSystem e4("e4");
    EndSystem e5("e5");
    EndSystem e6("e6");
    EndSystem e7("e7");
    EndSystem e8("e8");

    Switch sw1("sw1");
    Switch sw2("sw2");
    Switch sw3("sw3");
    Switch sw4("sw4");

    vector<EndSystem> esList {e1, e2, e3, e4, e5, e6, e7, e8};
    vector<Switch> swList {sw1, sw2, sw3, sw4};

    e1.setConnectedSwitch(sw1.getPorts()[0]);
    e2.setConnectedSwitch(sw1.getPorts()[1]);
    e3.setConnectedSwitch(sw2.getPorts()[0]);
    e4.setConnectedSwitch(sw2.getPorts()[1]);
    e5.setConnectedSwitch(sw3.getPorts()[0]);
    e6.setConnectedSwitch(sw3.getPorts()[1]);
    e7.setConnectedSwitch(sw4.getPorts()[0]);
    e8.setConnectedSwitch(sw4.getPorts()[1]);

    sw1.setConnectedES(e1, 0);
    sw1.setConnectedES(e2, 1);

    sw2.setConnectedES(e3, 0);
    sw2.setConnectedES(e4, 1);

    sw3.setConnectedES(e5, 0);
    sw3.setConnectedES(e6, 1);

    sw4.setConnectedES(e7, 0);
    sw4.setConnectedES(e8, 1);

    sw1.setConnectedSW(sw1.getPorts()[3], sw2.getPorts()[3]);
    sw1.setConnectedSW(sw1.getPorts()[4], sw3.getPorts()[3]);

    sw2.setConnectedSW(sw2.getPorts()[3], sw1.getPorts()[3]);
    sw2.setConnectedSW(sw2.getPorts()[4], sw4.getPorts()[3]);

    sw3.setConnectedSW(sw3.getPorts()[3], sw1.getPorts()[4]);
    sw3.setConnectedSW(sw3.getPorts()[4], sw4.getPorts()[4]);

    sw4.setConnectedSW(sw4.getPorts()[3], sw2.getPorts()[4]);
    sw4.setConnectedSW(sw4.getPorts()[4], sw3.getPorts()[4]);


    std::vector<Flow> flows;
    for (int i = 0; i < 1000; ++i) {
        Flow flow(0, genRandFramePeriod(), genRandFrameLen(), e1, e2);
        flows.push_back(flow);
    }
    long long hyperPeriod = getHyperPeriod(flows);
    std::cout << "hyperPeriod = " << hyperPeriod << std::endl;
    std::cout << "Whole send delay : " << getWholeSendInterval(flows) << std::endl;
    return 0;
}
