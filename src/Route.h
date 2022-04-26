//
// Created by faniche on 2022/3/16.
//

#ifndef SCHEDPLUS_ROUTE_H
#define SCHEDPLUS_ROUTE_H

#include <vector>
#include <sstream>
#include "components/Node.h"
#include "components/Link.h"

/*
 * https://www.geeksforgeeks.org/find-paths-given-source-destination/
 * */

// A directed graph using
// adjacency list representation
class Graph {
private:
    size_t V; // No. of vertices in graph
    std::vector<std::vector<size_t>> adj; // Pointer to an array containing adjacency lists

    // A recursive function used by getAllRoutes()
    void printAllPathsUtil(size_t, size_t, std::vector<bool> &, std::vector<size_t> &, size_t &,
                           std::vector<std::vector<size_t>> &routes);

public:
    explicit Graph(size_t V); // Constructor
    void addEdge(size_t u, size_t v);

    void getAllRoutes(size_t s, size_t d, std::vector<std::vector<size_t>> &routes);

    static void initGraph(std::map<size_t, Node *> &map, const std::vector<DirectedLink> &links, Graph &graph);
};

class Route {
private:
    /* The set of links of a Route. */
//    std::vector<DirectedLink *> links;
    std::vector<std::reference_wrapper<DirectedLink>> links;

    /* End to end latency except queue delay */
    uint32_t e2e = 0;

public:
    explicit Route();

//    [[nodiscard]] const std::vector<DirectedLink *> &getLinks() const;

    [[nodiscard]] const std::vector<std::reference_wrapper<DirectedLink>> &getLinks() const;

    void addLink(DirectedLink &link);

    [[nodiscard]] uint32_t getE2E() const;

    void setE2E(uint32_t e2E);
};


#endif //SCHEDPLUS_ROUTE_H
