#include "GraphGenerator.h"
#include "Utils.h"

GraphData gerar_lista_de_adjacencia(const std::vector<std::vector<std::string>>& grid) {
    int rows = grid.size();
    int cols = grid[0].size();
    
    // Mapear células livres para índices de vértices
    std::map<std::pair<int,int>, int> pos_to_index;
    std::map<int, std::pair<int,int>> index_to_pos;
    int current = 0;
    int robot_idx = -1;
    int target_idx = -1;
    
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            std::string cell = to_lower(grid[i][j]);
            
            if (cell == "b") continue;
            
            // Atribuir índice à célula livre
            pos_to_index[{i, j}] = current;
            index_to_pos[current] = {i, j};
            
            if (cell == "r") {
                robot_idx = current;
            }
            else if (cell == "t") {
                target_idx = current;
            }
            
            current++;
        }
    }
    
    int N = current;  // Número total de vértices (células não-bloqueadas)
    GraphData graph_data(N);
    graph_data.pos_to_index = pos_to_index;
    graph_data.index_to_pos = index_to_pos;
    graph_data.robot_index = robot_idx;
    graph_data.target_index = target_idx;
    
    // Adicionar arestas entre células adjacentes
    // Direções: cima, baixo, esquerda, direita
    int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    // Para cada célula livre no grid
    for (const auto& [pos, u] : pos_to_index) {
        int i = pos.first;
        int j = pos.second;
        
        // Verificar cada uma das 4 direções
        for (int d = 0; d < 4; d++) {
            int ni = i + dirs[d][0];
            int nj = j + dirs[d][1];
            
            // Verificar se a nova posição está dentro dos limites
            if (ni < 0 || ni >= rows || nj < 0 || nj >= cols) continue;
            
            if (to_lower(grid[ni][nj]) == "b") continue;
            
            // Obter o índice do vértice vizinho
            auto it = pos_to_index.find({ni, nj});
            if (it != pos_to_index.end()) {
                int v = it->second;
                // Adicionar aresta com peso 1
                graph_data.adj_list.add_edge(u, v, 1);
            }
        }
    }
    
    return graph_data;
}
