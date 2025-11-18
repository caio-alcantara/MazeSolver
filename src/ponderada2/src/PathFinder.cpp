#include "PathFinder.h"
#include <queue>
#include <unordered_map>
#include <algorithm>

std::vector<int> PathFinder::find_shortest_path(
    const GraphData& graph_data,
    int start,
    int goal
) {
    std::vector<int> path;
    
    // Validação de entrada
    if (start < 0 || goal < 0 || start >= graph_data.adj_list.get_num_vertices() 
        || goal >= graph_data.adj_list.get_num_vertices()) {
        return path;
    }
    
    if (start == goal) {
        path.push_back(start);
        return path;
    }
    
    // BFS para encontrar o caminho mais curto
    std::queue<int> queue;
    std::unordered_map<int, int> parent;
    std::unordered_map<int, bool> visited;
    
    queue.push(start);
    visited[start] = true;
    parent[start] = -1;
    
    bool found = false;
    
    while (!queue.empty() && !found) {
        int current = queue.front();
        queue.pop();
        
        if (current == goal) {
            found = true;
            break;
        }
        
        // Explora vizinhos
        const auto& neighbors = graph_data.adj_list.get_neighbors(current);
        for (const auto& [neighbor, weight] : neighbors) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                queue.push(neighbor);
            }
        }
    }
    
    // Reconstrói o caminho se encontrado
    if (found) {
        int current = goal;
        while (current != -1) {
            path.push_back(current);
            current = parent[current];
        }
        std::reverse(path.begin(), path.end());
    }
    
    return path;
}

std::vector<std::pair<int, int>> PathFinder::path_to_coordinates(
    const GraphData& graph_data,
    const std::vector<int>& path
) {
    std::vector<std::pair<int, int>> coordinates;
    
    for (int vertex : path) {
        auto it = graph_data.index_to_pos.find(vertex);
        if (it != graph_data.index_to_pos.end()) {
            coordinates.push_back(it->second);
        }
    }
    
    return coordinates;
}
