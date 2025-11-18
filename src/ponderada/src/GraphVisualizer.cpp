#include "GraphVisualizer.h"
#include <iostream>
#include <algorithm>

void GraphVisualizer::print_adjacency_list(
    const GraphData& graph_data,
    int max_vertices
) {
    std::cout << "LISTA DE ADJACÊNCIA (primeiros " << max_vertices << " vértices)" << std::endl;
    
    int num_to_print = std::min(max_vertices, graph_data.adj_list.get_num_vertices());
    for (int i = 0; i < num_to_print; i++) {
        std::cout << i << ": ";
        bool first = true;
        for (const auto& [neighbor, weight] : graph_data.adj_list.get_neighbors(i)) {
            if (!first) std::cout << ", ";
            std::cout << neighbor;
            first = false;
        }
        std::cout << std::endl;
    }
}

void GraphVisualizer::print_path(
    const std::vector<std::pair<int, int>>& path
) {
    if (path.empty()) {
        std::cout << "Nenhum caminho encontrado." << std::endl;
        return;
    }

    std::cout << "CAMINHO ENCONTRADO" << std::endl;
    std::cout << "Comprimento: " << path.size() << " células" << std::endl;
    std::cout << "Caminho: ";
    
    for (size_t i = 0; i < path.size(); i++) {
        std::cout << "(" << path[i].first << "," << path[i].second << ")";
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
}
