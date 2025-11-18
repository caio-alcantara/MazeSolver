#ifndef GRAPH_GENERATOR_H
#define GRAPH_GENERATOR_H

#include "GraphData.h"
#include <vector>
#include <string>

/**
 * @brief Converte o grid do labirinto em um grafo (lista de adjacência)
 * 
 * Algoritmo:
 * 1. Percorre o grid e atribui um índice único para cada célula não-bloqueada (!= b)
 * 2. Identifica as posições do robô (R) e do alvo (T)
 * 3. Para cada célula livre, verifica seus 4 vizinhos (cima, baixo, esquerda, direita)
 * 4. Cria arestas entre células adjacentes que não são bloqueadas
 * 
 * Células bloqueadas (b) são ignoradas e não recebem índice de vértice.
 * 
 * @param grid Matriz 2D representando o labirinto
 * @return GraphData contendo o grafo e todos os mapeamentos necessários
 */
GraphData gerar_lista_de_adjacencia(const std::vector<std::vector<std::string>>& grid);

#endif // GRAPH_GENERATOR_H
