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

Route::Route() {}

const std::vector<DirectedLink *> &Route::getLinks() const {
    return links;
}

void Route::setLinks(const std::vector<DirectedLink *> &_links) {
    Route::links = _links;
}

int Route::getE2E() const {
    return e2e;
}

void Route::setE2E(int e2E) {
    e2e = e2E;
}

/**
 * @brief Get all routes of flow
 * @param map   : node index map
 * @param flow  : flow waiting for calculation all routes
 * @param graph : Integer adjacency matrix of all node
 * @param alllinks : all alllinks vector
 **/
void Route::calAllRoutes(std::map<size_t, Node *> &map, Flow &flow, Graph &graph, std::vector<DirectedLink> &alllinks) {
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
        Route route;
        int tmp = 0;
        for (int j = 1; j < idxRoute.size(); ++j) {
            destIdx = idxRoute[j];
            DirectedLink *link1 = DirectedLink::nodesIdxToLink(map.at(srcIdx), map.at(destIdx), alllinks);
            srcIdx = destIdx;
            route.links.push_back(link1);
            /* Calculate the e2e latency except queue delay of flow. */
            int trans_delay = flow.getFrameLength() * link1->getSrcPort().getMacrotick();
            int proc_delay = link1->getSrcNode()->getDpr();
            int prop_delay = link1->getLen() * link1->getPropSpeed();
            tmp += trans_delay + proc_delay + prop_delay;
        }
        route.setE2E(tmp);
        flow.addRoutes(route);
    }

}
