#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#include "GraphData.h"
#include <vector>

/**
 * @brief Classe responsável por encontrar caminhos no grafo
 * 
 * Implementa algoritmos de busca para encontrar o caminho
 * entre o robô e o alvo no labirinto.
 */
class PathFinder {
public:
    /**
     * @brief Encontra o caminho mais curto entre dois vértices usando BFS
     * @param graph_data Dados do grafo
     * @param start Índice do vértice inicial
     * @param goal Índice do vértice objetivo
     * @return Vetor com os índices dos vértices no caminho (vazio se não houver caminho)
     */
    static std::vector<int> find_shortest_path(
        const GraphData& graph_data,
        int start,
        int goal
    );

    /**
     * @brief Converte um caminho de índices para coordenadas do grid
     * @param graph_data Dados do grafo
     * @param path Caminho em índices de vértices
     * @return Vetor de pares (linha, coluna) representando o caminho no grid
     */
    static std::vector<std::pair<int, int>> path_to_coordinates(
        const GraphData& graph_data,
        const std::vector<int>& path
    );
};

#endif // PATH_FINDER_H
