#ifndef ROS_MAP_CLIENT_H
#define ROS_MAP_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include <vector>
#include <string>

/**
 * @brief Chama o serviço /get_map e retorna o mapa como matriz 2D
 * 
 * Esta função:
 * 1. Aguarda o serviço /get_map estar disponível
 * 2. Envia uma requisição para obter o mapa
 * 3. Converte o mapa achatado em matriz 2D
 * 
 * @param node Nó ROS 2
 * @param client Cliente do serviço /get_map
 * @return Matriz 2D representando o mapa (vazia em caso de erro)
 */
std::vector<std::vector<std::string>> obter_mapa(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client
);

#endif // ROS_MAP_CLIENT_H
