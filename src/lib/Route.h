//
// Created by faniche on 2022/3/13.
//

#ifndef SCHEDPLUS_ROUTE_H
#define SCHEDPLUS_ROUTE_H

#include <vector>
#include <iostream>

/*
 * https://www.geeksforgeeks.org/find-paths-given-source-destination/
 * */

// A directed graph using
// adjacency list representation
class Graph {
    int V; // No. of vertices in graph
    std::vector<std::vector<int>> adj; // Pointer to an array containing adjacency lists

    // A recursive function used by printAllPaths()
    void printAllPathsUtil(int, int, std::vector<bool>&, std::vector<int>&, int &, std::vector<std::vector<int>> &routs);

public:
    explicit Graph(int V); // Constructor
    void addEdge(int u, int v);

    void printAllPaths(int s, int d, std::vector<std::vector<int>> &routs);
};

Graph::Graph(int V) {
    this->V = V;
    for (int i = 0; i < V; ++i) {
        std::vector<int> tmp{i};
        adj.push_back(tmp);
    }
}

void Graph::addEdge(int u, int v) {
    adj[u].push_back(v); // Add v to u’s list.
}

// Prints all paths from 's' to 'd'
void Graph::printAllPaths(int s, int d, std::vector<std::vector<int>> &routs) {
    // Mark all the vertices as not visited
//    bool *visited = new bool[V];
    std::vector<bool> visited(V, false);

    // Create an array to store paths
//    int *path = new int[V];
    std::vector<int> path(V);
    int path_index = 0; // Initialize path[] as empty

    // Initialize all vertices as not visited
//    for (int i = 0; i < V; i++)
//        visited[i] = false;

    // Call the recursive helper function to print all paths
    printAllPathsUtil(s, d, visited, path, path_index, routs);
}

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void Graph::printAllPathsUtil(int u, int d,
                              std::vector<bool> &visited,
                              std::vector<int> &path,
                              int &path_index,
                              std::vector<std::vector<int>> &routs) {
    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    // If current vertex is same as destination, then print
    // current path[]
    if (u == d) {
        std::vector<int> tmp;
        for (int i = 0; i < path_index; i++){
            std::cout << path[i] << " ";
            tmp.push_back(path[i]);
        }
        routs.push_back(tmp);
        std::cout << std::endl;
    } else // If current vertex is not destination
    {
        // Recur for all the vertices adjacent to current vertex
        std::vector<int>::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
            if (!visited[*i])
                printAllPathsUtil(*i, d, visited, path, path_index, routs);
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[u] = false;
}


#endif //SCHEDPLUS_ROUTE_H
