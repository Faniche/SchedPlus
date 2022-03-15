//
// Created by faniche on 2022/3/13.
//

#ifndef SCHEDPLUS_ROUTE_H
#define SCHEDPLUS_ROUTE_H

#include <vector>
#include <sstream>

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
};

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


#endif //SCHEDPLUS_ROUTE_H
