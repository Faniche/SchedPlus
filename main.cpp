#include <map>
#include <vector>
#include <spdlog/spdlog.h>
#include "src/Utils.h"
#include "src/components/Node.h"

using std::vector;

int main() {
    std::ostringstream oss;
    /* End systems */
    Node *es0 = createNode(END_SYSTEM, "es0");
    Node *es1 = createNode(END_SYSTEM, "es1");
    Node *es2 = createNode(END_SYSTEM, "es2");
    Node *es3 = createNode(END_SYSTEM, "es3");
    Node *es4 = createNode(END_SYSTEM, "es4");
    Node *es5 = createNode(END_SYSTEM, "es5");
    Node *es6 = createNode(END_SYSTEM, "es6");
    Node *es7 = createNode(END_SYSTEM, "es7");
    /* Switches */
    Node *sw0 = createNode(SWITCH, "sw0");
    Node *sw1 = createNode(SWITCH, "sw1");
    Node *sw2 = createNode(SWITCH, "sw2");
    Node *sw3 = createNode(SWITCH, "sw3");
    Node *sw4 = createNode(SWITCH, "sw4");
    vector<Node *> nodes{es0, es1, es2, es3, es4, es5, es6, es7, sw0, sw1, sw2, sw3, sw4};

    std::map<size_t, Node*> map;
    for (size_t i = 0; i < nodes.size(); ++i) {
        map[i] = nodes[i];
    }

    /* Links connected end systems to switches */
    FullDuplexLink link_00(es0, sw0, ((EndSystem *) es0)->getPort(), ((Switch *) sw0)->getPorts().at(0));
    FullDuplexLink link_01(es1, sw0, ((EndSystem *) es1)->getPort(), ((Switch *) sw0)->getPorts().at(1));
    FullDuplexLink link_02(es2, sw1, ((EndSystem *) es2)->getPort(), ((Switch *) sw1)->getPorts().at(0));
    FullDuplexLink link_03(es3, sw1, ((EndSystem *) es3)->getPort(), ((Switch *) sw1)->getPorts().at(1));
    FullDuplexLink link_04(es4, sw2, ((EndSystem *) es4)->getPort(), ((Switch *) sw2)->getPorts().at(0));
    FullDuplexLink link_05(es5, sw2, ((EndSystem *) es5)->getPort(), ((Switch *) sw2)->getPorts().at(1));
    FullDuplexLink link_06(es6, sw3, ((EndSystem *) es6)->getPort(), ((Switch *) sw3)->getPorts().at(0));
    FullDuplexLink link_07(es7, sw3, ((EndSystem *) es7)->getPort(), ((Switch *) sw3)->getPorts().at(1));
    /* Links connected four switches */
    FullDuplexLink link_08(sw0, sw1, ((Switch *) sw0)->getPorts().at(2), ((Switch *) sw1)->getPorts().at(2));
    FullDuplexLink link_09(sw0, sw2, ((Switch *) sw0)->getPorts().at(3), ((Switch *) sw2)->getPorts().at(2));
    FullDuplexLink link_10(sw2, sw3, ((Switch *) sw2)->getPorts().at(3), ((Switch *) sw3)->getPorts().at(2));
    FullDuplexLink link_11(sw1, sw3, ((Switch *) sw1)->getPorts().at(3), ((Switch *) sw3)->getPorts().at(3));

    FullDuplexLink link_12(sw0, sw4, ((Switch *) sw0)->getPorts().at(4), ((Switch *) sw4)->getPorts().at(0));
    FullDuplexLink link_13(sw1, sw4, ((Switch *) sw1)->getPorts().at(4), ((Switch *) sw4)->getPorts().at(1));
    FullDuplexLink link_14(sw2, sw4, ((Switch *) sw2)->getPorts().at(4), ((Switch *) sw4)->getPorts().at(2));
    FullDuplexLink link_15(sw3, sw4, ((Switch *) sw3)->getPorts().at(4), ((Switch *) sw4)->getPorts().at(3));

    /* Add links to vectors */
    vector<DirectedLink> links{link_00.getLinks()[0], link_00.getLinks()[1],
                               link_01.getLinks()[0], link_01.getLinks()[1],
                               link_02.getLinks()[0], link_02.getLinks()[1],
                               link_03.getLinks()[0], link_03.getLinks()[1],
                               link_04.getLinks()[0], link_04.getLinks()[1],
                               link_05.getLinks()[0], link_05.getLinks()[1],
                               link_06.getLinks()[0], link_06.getLinks()[1],
                               link_07.getLinks()[0], link_07.getLinks()[1],
                               link_08.getLinks()[0], link_08.getLinks()[1],
                               link_09.getLinks()[0], link_09.getLinks()[1],
                               link_10.getLinks()[0], link_10.getLinks()[1],
                               link_11.getLinks()[0], link_11.getLinks()[1],
                               link_12.getLinks()[0], link_12.getLinks()[1],
                               link_13.getLinks()[0], link_13.getLinks()[1],
                               link_14.getLinks()[0], link_14.getLinks()[1],
                               link_15.getLinks()[0], link_15.getLinks()[1]};

    /* Adjacency matrix */
    /* Init a graph of int value to calculate routes later*/
    Graph graph(nodes.size());
    Graph::initGraph(map, links, graph);

    /* The set of flows */
    std::vector<Flow> flows;
    /* es0 ----> es2 */
    Flow flow_01(genRandFramePeriod(), 10, genRandFrameLen(), es0, es2, false, 1, false);
    flow_01.setPriorityCodePoint(VO);
    Route::getRoutes(map, flow_01, graph, links);
//    graph.getAllRoutes(0, 7, routes);


    flows.push_back(flow_01);
//    flow_01.
//    getRoutes(links, nodesAdjacencyMatrix, *es0, *es2);

    spdlog::info("flow_01:" + flow_01.toString(oss));
    oss.clear();


    delete es0;
    delete es1;
    delete es2;
    delete es3;
    delete es4;
    delete es5;
    delete es6;
    delete es7;

    delete sw0;
    delete sw1;
    delete sw2;
    delete sw3;

    return 0;
}
