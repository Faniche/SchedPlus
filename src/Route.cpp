//
// Created by faniche on 2022/3/16.
//

#include <spdlog/spdlog.h>
#include "Route.h"

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void Graph::printAllPathsUtil(size_t u, size_t d,
                              std::vector<bool> &visited,
                              std::vector<size_t> &path,
                              size_t &path_index,
                              std::vector<std::vector<size_t>> &routes) {
    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    /* If current vertex is same as destination, then store current path[] */
    if (u == d) {
        std::vector<size_t> tmp;
        for (size_t i = 0; i < path_index; i++)
            tmp.push_back(path[i]);
        routes.push_back(tmp);
    } else {
        /* If current vertex is not destination */
        /* Recur for all the vertices adjacent to current vertex */
        std::vector<size_t>::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
            if (!visited[*i])
                printAllPathsUtil(*i, d, visited, path, path_index, routes);
    }
    /* Remove current vertex from path[] and mark it as unvisited */
    path_index--;
    visited[u] = false;
}

Graph::Graph(size_t V) {
    this->V = V;
    for (size_t i = 0; i < V; ++i) {
        std::vector<size_t> tmp{i};
        adj.push_back(tmp);
    }
}

void Graph::addEdge(size_t u, size_t v) {
    adj[u].push_back(v); // Add v to u’s list.
}

// Prints all paths from 's' to 'd'
void Graph::getAllRoutes(size_t s, size_t d, std::vector<std::vector<size_t>> &routes) {
    // Initialize all vertices as not visited
    std::vector<bool> visited(V, false);

    // Create an array to store paths
    std::vector<size_t> path(V);

    size_t path_index = 0;

    // Call the recursive helper function to print all paths
    printAllPathsUtil(s, d, visited, path, path_index, routes);
}

/**
 * @brief Init the Integer type of graph
 * @param map       : reference to node index map
 * @param links     : link vector
 * @param graph     : integer adjacency matrix
 **/
void Graph::initGraph(std::map<size_t, Node *> &map, const std::vector<DirectedLink> &links, Graph &graph) {
    for (auto &link: links) {
        graph.addEdge(Node::nodeToIdx(map, link.getSrcNode()), Node::nodeToIdx(map, link.getDestNode()));
    }
}

/**
 * @brief Get all routes of flow
 * @param map   : node index map
 * @param flow  : flow waiting for calculation all routes
 * @param graph : Integer adjacency matrix of all node
 * @param links : links vector
 **/
void Route::getRoutes(std::map<size_t, Node *> &map, Flow &flow, Graph &graph, std::vector<DirectedLink> &links) {
    size_t srcIdx = Node::nodeToIdx(map, flow.getSrc());
    if (srcIdx == INT64_MAX)
        spdlog::error("can not find the index of node: %s", flow.getSrc()->getName());
    size_t destIdx = Node::nodeToIdx(map, flow.getDest());
    if (destIdx == INT64_MAX)
        spdlog::error("can not find the index of node: %s", flow.getDest()->getName());
    std::vector<std::vector<size_t>> routes;
    graph.getAllRoutes(srcIdx, destIdx, routes);
    for (auto &idxRoute: routes) {
        srcIdx = idxRoute[0];
        std::vector<DirectedLink *> route;
//        std::vector<Node *> nodeVector;
//        nodeVector.push_back(flow.getSrc());
        for (int j = 1; j < idxRoute.size(); ++j) {
            destIdx = idxRoute[j];
            DirectedLink *link1 = DirectedLink::nodesIdxToLink(map.at(srcIdx), map.at(destIdx), links);
//            nodeVector.push_back(map[destIdx]);
            srcIdx = destIdx;
            route.push_back(link1);
        }
        flow.setRoutes(route);
//        flow.setRoutesAdj(nodeVector);
    }

}
