#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Converte uma string para minúsculas
 * @param str String a ser convertida
 * @return String em minúsculas
 */
std::string to_lower(const std::string& str);

/**
 * @brief Converte um array 1D em uma matriz 2D
 * 
 * O serviço /get_map retorna o mapa como um array achatado (1D).
 * Esta função reconstrói a matriz 2D original.
 * 
 * @param flat Array 1D com os dados do mapa
 * @param altura Número de linhas da matriz
 * @param largura Número de colunas da matriz
 * @return Matriz 2D representando o mapa
 */
std::vector<std::vector<std::string>> converter_para_matriz_2d(
    const std::vector<std::string> &flat,
    int altura,
    int largura
);

/**
 * @brief Imprime o mapa do labirinto de forma formatada
 * @param matriz Matriz 2D representando o mapa
 * @param logger Logger do ROS 2 para saída
 */
void imprimir_mapa(
    const std::vector<std::vector<std::string>> &matriz,
    const rclcpp::Logger &logger
);

#endif // UTILS_H
