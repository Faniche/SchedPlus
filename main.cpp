#include <iostream>
#include <vector>
#include <sstream>
#include "Components.h.bak"
#include "src/Utils.h"
#include "src/lib/components/Node.h"
#include "src/lib/components/NodeImpl.h"
#include "src/lib/components/Link.h"
#include "src/lib/components/Flow.h"

using std::cout;    using std::endl;
using std::vector;

int main() {
//    EndSystem e1("e1");
//    EndSystem e2("e2");
//    EndSystem e3("e3");
//    EndSystem e4("e4");
//    EndSystem e5("e5");
//    EndSystem e6("e6");
//    EndSystem e7("e7");
//    EndSystem e8("e8");
//
//    Switch sw1("sw1");
//    Switch sw2("sw2");
//    Switch sw3("sw3");
//    Switch sw4("sw4");
//
//    vector<EndSystem> esList {e1, e2, e3, e4, e5, e6, e7, e8};
//    vector<Switch> swList {sw1, sw2, sw3, sw4};
//
//    e1.setConnectedSwitch(sw1.getPorts()[0]);
//    e2.setConnectedSwitch(sw1.getPorts()[1]);
//    e3.setConnectedSwitch(sw2.getPorts()[0]);
//    e4.setConnectedSwitch(sw2.getPorts()[1]);
//    e5.setConnectedSwitch(sw3.getPorts()[0]);
//    e6.setConnectedSwitch(sw3.getPorts()[1]);
//    e7.setConnectedSwitch(sw4.getPorts()[0]);
//    e8.setConnectedSwitch(sw4.getPorts()[1]);
//
//    sw1.setConnectedES(e1, 0);
//    sw1.setConnectedES(e2, 1);
//
//    sw2.setConnectedES(e3, 0);
//    sw2.setConnectedES(e4, 1);
//
//    sw3.setConnectedES(e5, 0);
//    sw3.setConnectedES(e6, 1);
//
//    sw4.setConnectedES(e7, 0);
//    sw4.setConnectedES(e8, 1);
//
//    sw1.setConnectedSW(sw1.getPorts()[3], sw2.getPorts()[3]);
//    sw1.setConnectedSW(sw1.getPorts()[4], sw3.getPorts()[3]);
//
//    sw2.setConnectedSW(sw2.getPorts()[3], sw1.getPorts()[3]);
//    sw2.setConnectedSW(sw2.getPorts()[4], sw4.getPorts()[3]);
//
//    sw3.setConnectedSW(sw3.getPorts()[3], sw1.getPorts()[4]);
//    sw3.setConnectedSW(sw3.getPorts()[4], sw4.getPorts()[4]);
//
//    sw4.setConnectedSW(sw4.getPorts()[3], sw2.getPorts()[4]);
//    sw4.setConnectedSW(sw4.getPorts()[4], sw3.getPorts()[4]);
//
//
//    std::vector<Flow> flows;
//    for (int i = 0; i < 1000; ++i) {
//        Flow flow(0, genRandFramePeriod(), genRandFrameLen(), e1, e2);
//        flows.push_back(flow);
//    }
//    long long hyperPeriod = getHyperPeriod(flows);
//    std::cout << "hyperPeriod = " << hyperPeriod << std::endl;
//    std::cout << "Whole send delay : " << getWholeSendInterval(flows) << std::endl;

    std::ostringstream oss;

    Node *es1 = createNode(END_SYSTEM, "es1");
    Node *es2 = createNode(END_SYSTEM, "es2");
    Node *es3 = createNode(END_SYSTEM, "es3");
    Node *es4 = createNode(END_SYSTEM, "es4");
    Node *es5 = createNode(END_SYSTEM, "es5");
    Node *es6 = createNode(END_SYSTEM, "es6");
    Node *es7 = createNode(END_SYSTEM, "es7");
    Node *es8 = createNode(END_SYSTEM, "es8");

    Node *sw1 = createNode(SWITCH, "sw1");
    Node *sw2 = createNode(SWITCH, "sw2");
    Node *sw3 = createNode(SWITCH, "sw3");
    Node *sw4 = createNode(SWITCH, "sw4");

    Link link_01(((EndSystem *) es1)->getPort(), ((Switch *) sw1)->getPorts().at(0));
    Link link_02(((EndSystem *) es2)->getPort(), ((Switch *) sw1)->getPorts().at(1));

    Link link_03(((EndSystem *) es3)->getPort(), ((Switch *) sw2)->getPorts().at(0));
    Link link_04(((EndSystem *) es4)->getPort(), ((Switch *) sw2)->getPorts().at(1));

    Link link_05(((EndSystem *) es5)->getPort(), ((Switch *) sw3)->getPorts().at(0));
    Link link_06(((EndSystem *) es6)->getPort(), ((Switch *) sw3)->getPorts().at(1));

    Link link_07(((EndSystem *) es7)->getPort(), ((Switch *) sw4)->getPorts().at(0));
    Link link_08(((EndSystem *) es8)->getPort(), ((Switch *) sw4)->getPorts().at(1));


    Link link_09(((Switch *) sw1)->getPorts().at(2), ((Switch *) sw2)->getPorts().at(2));
    Link link_10(((Switch *) sw1)->getPorts().at(3), ((Switch *) sw3)->getPorts().at(2));
    Link link_11(((Switch *) sw3)->getPorts().at(3), ((Switch *) sw4)->getPorts().at(2));
    Link link_12(((Switch *) sw2)->getPorts().at(3), ((Switch *) sw4)->getPorts().at(3));

    /**
     * es1 ----> es3
     */
    Flow flow_01(genRandFramePeriod(), genRandFrameLen(), es1, es3, false, 1, false);
    cout << flow_01.toString(oss) << endl;
    oss.clear();

    delete es1;
    delete es2;
    delete es3;
    delete es4;
    delete es5;
    delete es6;
    delete es7;
    delete es8;

    delete sw1;
    delete sw2;
    delete sw3;
    delete sw4;

    return 0;
}
