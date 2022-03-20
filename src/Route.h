//
// Created by faniche on 2022/3/16.
//

#ifndef SCHEDPLUS_ROUTE_H
#define SCHEDPLUS_ROUTE_H

#include <vector>
#include <sstream>
#include "components/Node.h"
#include "components/Flow.h"

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
    void printAllPathsUtil(size_t, size_t, std::vector<bool>&, std::vector<size_t>&, size_t &, std::vector<std::vector<size_t>> &routes);

public:
    explicit Graph(size_t V); // Constructor
    void addEdge(size_t u, size_t v);

    void getAllRoutes(size_t s, size_t d, std::vector<std::vector<size_t>> &routes);

    static void initGraph(std::map<size_t, Node *> &map, const std::vector<DirectedLink> &links, Graph &graph);
};

class Route {
public:
    static void getRoutes(std::map<size_t, Node *> &map,
                   Flow &flow,
                   Graph &graph,
                   std::vector<DirectedLink> &links);
};


#endif //SCHEDPLUS_ROUTE_H
