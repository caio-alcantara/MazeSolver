#include "RosMapClient.h"
#include "Utils.h"
#include <chrono>

std::vector<std::vector<std::string>> obter_mapa(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client
) {
    std::vector<std::vector<std::string>> matriz_vazia;

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Aguardando o serviço /get_map estar disponível...");
    }

    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) 
        != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Falha ao chamar o serviço /get_map.");
        return matriz_vazia;
    }

    auto response = future.get();
    
    // Valida se o shape tem 2 dimensões (altura e largura)
    if (response->occupancy_grid_shape.size() != 2) {
        RCLCPP_ERROR(node->get_logger(), "Shape do mapa inválido.");
        return matriz_vazia;
    }

    // Extrai dimensões do mapa
    int altura = static_cast<int>(response->occupancy_grid_shape[0]);
    int largura = static_cast<int>(response->occupancy_grid_shape[1]);

    auto matriz = converter_para_matriz_2d(
        response->occupancy_grid_flattened,
        altura,
        largura
    );

    if (matriz.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Falha ao converter o mapa para matriz 2D.");
    }

    return matriz;
}
