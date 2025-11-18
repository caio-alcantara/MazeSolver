#ifndef GRAPH_VISUALIZER_H
#define GRAPH_VISUALIZER_H

#include "GraphData.h"
#include <vector>

/**
 * @brief Classe responsável por visualizar informações do grafo
 */
class GraphVisualizer {
public:
    /**
     * @brief Imprime informações básicas do grafo
     * @param graph_data Dados do grafo
     * @param rows Número de linhas do grid
     * @param cols Número de colunas do grid
     */
    static void print_graph_info(
        const GraphData& graph_data,
        int rows,
        int cols
    );

    /**
     * @brief Imprime a lista de adjacência
     * @param graph_data Dados do grafo
     * @param max_vertices Número máximo de vértices a imprimir
     */
    static void print_adjacency_list(
        const GraphData& graph_data,
        int max_vertices = 20
    );

    /**
     * @brief Imprime um caminho em formato de coordenadas
     * @param path Caminho em coordenadas (linha, coluna)
     */
    static void print_path(
        const std::vector<std::pair<int, int>>& path
    );
};

#endif // GRAPH_VISUALIZER_H
