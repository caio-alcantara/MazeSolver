#include "HashmapAdjacencyList.h"
#include <iostream>

HashmapAdjacencyList::HashmapAdjacencyList(int num_vertices, bool is_directed)
    : num_vertices(num_vertices), 
      is_directed(is_directed), 
      adj_map(num_vertices) { 
}

void HashmapAdjacencyList::print_list() const {
    for (int u = 0; u < num_vertices; ++u) {
        std::cout << u << ": ";
        for (const auto& [v, weight] : adj_map[u]) {
            std::cout << v;
            if (weight != 1) std::cout << "(" << weight << ")";
            std::cout << ", ";
        }
        std::cout << std::endl;
    }
}

void HashmapAdjacencyList::add_edge(int u, int v, int weight) {
    adj_map[u][v] = weight;
    if (!is_directed) {
        adj_map[v][u] = weight;
    }
}

bool HashmapAdjacencyList::has_adj(int u, int v) const {
    auto it = adj_map[u].find(v);
    return (it != adj_map[u].end() && it->second == 1);
}

int HashmapAdjacencyList::get_num_vertices() const {
    return num_vertices;
}

const std::unordered_map<int, int>& HashmapAdjacencyList::get_neighbors(int v) const {
    return adj_map[v];
}
