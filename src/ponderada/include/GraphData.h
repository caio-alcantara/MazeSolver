#ifndef GRAPH_DATA_H
#define GRAPH_DATA_H

#include "HashmapAdjacencyList.h"
#include <map>
#include <utility>

/**
 * @brief Estrutura que encapsula todas as informações do grafo gerado
 * 
 * Mantém não apenas a lista de adjacência, mas também os mapeamentos
 * bidirecionais entre posições do grid e índices dos vértices.
 */
struct GraphData {
    HashmapAdjacencyList adj_list;
    
    // Mapeamento: posição (linha, coluna) -> índice do vértice
    std::map<std::pair<int,int>, int> pos_to_index;
    
    // Mapeamento reverso: índice do vértice -> posição (linha, coluna)
    std::map<int, std::pair<int,int>> index_to_pos;
    
    int robot_index;
    int target_index;
    
    GraphData(int num_vertices) 
        : adj_list(num_vertices, false), 
          robot_index(-1), 
          target_index(-1) {}
};

#endif // GRAPH_DATA_H
