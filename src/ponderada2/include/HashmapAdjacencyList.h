#ifndef HASHMAP_ADJACENCY_LIST_H
#define HASHMAP_ADJACENCY_LIST_H

#include <vector>
#include <unordered_map>

/**
 * @brief Representa um grafo usando lista de adjacência implementada com hashmap.
 * 
 * Esta estrutura permite armazenar grafos direcionados ou não-direcionados,
 * onde cada vértice mantém um mapa de seus vizinhos e os pesos das arestas.
 */
class HashmapAdjacencyList {
private:
    int num_vertices;
    bool is_directed;
    // Cada posição do vetor representa um vértice
    // O unordered_map armazena: [vizinho -> peso da aresta]
    std::vector<std::unordered_map<int, int>> adj_map;
    
public:
    /**
     * @brief Construtor da lista de adjacência
     * @param num_vertices Número total de vértices no grafo
     * @param is_directed Se true, o grafo é direcionado; se false, não-direcionado
     */
    HashmapAdjacencyList(int num_vertices, bool is_directed = false);

    /**
     * @brief Imprime a lista de adjacência no formato: "vértice: vizinho1, vizinho2"
     */
    void print_list() const;

    /**
     * @brief Adiciona uma aresta entre dois vértices
     * @param u Vértice de origem
     * @param v Vértice de destino
     * @param weight Peso da aresta
     */
    void add_edge(int u, int v, int weight);

    /**
     * @brief Verifica se existe uma aresta entre dois vértices
     * @param u Vértice de origem
     * @param v Vértice de destino
     * @return true se existe aresta com peso 1, false caso contrário
     */
    bool has_adj(int u, int v) const;

    /**
     * @brief Retorna o número de vértices no grafo
     */
    int get_num_vertices() const;

    /**
     * @brief Retorna os vizinhos de um vértice específico
     * @param v Índice do vértice
     * @return Mapa de vizinhos e seus pesos
     */
    const std::unordered_map<int, int>& get_neighbors(int v) const;
};

#endif // HASHMAP_ADJACENCY_LIST_H
