#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "GraphGenerator.h"
#include "RosMapClient.h"
#include "GraphVisualizer.h"
#include "PathFinder.h"
#include <iostream>
#include <chrono>
#include <thread>

/**
 * @brief Converte uma sequência de coordenadas em comandos de direção
 * 
 * @param coordinates Vetor de pares (linha, coluna) representando o caminho
 * @return std::vector<std::string> Lista de comandos ("up", "down", "left", "right")
 */
std::vector<std::string> coordinates_to_commands(
    const std::vector<std::pair<int, int>>& coordinates
) {
    std::vector<std::string> commands;
    
    for (size_t i = 0; i < coordinates.size() - 1; i++) {
        int current_row = coordinates[i].first;
        int current_col = coordinates[i].second;
        int next_row = coordinates[i + 1].first;
        int next_col = coordinates[i + 1].second;
        
        // Calcula a diferença de posição
        int delta_row = next_row - current_row;
        int delta_col = next_col - current_col;
        
        // Determina o comando baseado na diferença
        if (delta_row == -1 && delta_col == 0) {
            commands.push_back("up");
        } else if (delta_row == 1 && delta_col == 0) {
            commands.push_back("down");
        } else if (delta_row == 0 && delta_col == -1) {
            commands.push_back("left");
        } else if (delta_row == 0 && delta_col == 1) {
            commands.push_back("right");
        } else {
            std::cerr << "ERRO: Movimento inválido detectado entre (" 
                      << current_row << "," << current_col << ") e (" 
                      << next_row << "," << next_col << ")" << std::endl;
        }
    }
    
    return commands;
}

/**
 * @brief Executa os comandos de movimento chamando o serviço /move_command
 * 
 * @param node Nó ROS 2
 * @param commands Lista de comandos a serem executados
 * @param delay_ms Delay em milissegundos entre cada comando (padrão: 300ms)
 * @return true se todos os comandos foram executados com sucesso
 */
bool execute_robot_movements(
    rclcpp::Node::SharedPtr node,
    const std::vector<std::string>& commands,
    int delay_ms = 300
) {
    auto move_client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    
    std::cout << "EXECUTANDO MOVIMENTOS NO ROBÔ" << std::endl;
    std::cout << "Total de movimentos: " << commands.size() << std::endl;
    
    std::cout << "\nAguardando serviço /move_command..." << std::endl;
    while (!move_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrompido enquanto aguardava o serviço");
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "Esperando serviço /move_command...");
    }
    
    int successful_moves = 0;
    int failed_moves = 0;
    
    for (size_t i = 0; i < commands.size(); i++) {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = commands[i];
        
        std::cout << "[" << (i + 1) << "/" << commands.size() << "] " 
                  << commands[i] << " ... ";
        std::cout.flush();
        
        auto future = move_client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node, future) 
            == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            
            if (response->success) {
                std::cout << "Sucesso" << std::endl;
                successful_moves++;
            } else {
                std::cout << "Falhou" << std::endl;
                failed_moves++;
                RCLCPP_WARN(node->get_logger(), "Movimento falhou, continuando...");
            }
        } else {
            std::cout << "Erro na chamada do serviço" << std::endl;
            failed_moves++;
            RCLCPP_ERROR(node->get_logger(), "Falha ao chamar /move_command");
        }
        
        if (i < commands.size() - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    
    return failed_moves == 0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("main");
    auto client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");

    // Obter o mapa do labirinto
    auto mapa = obter_mapa(node, client);
    if (mapa.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Não foi possível obter o mapa");
        rclcpp::shutdown();
        return 1;
    }

    // Converter o mapa em grafo
    auto graph_data = gerar_lista_de_adjacencia(mapa);
    
    // Exibir informações do grafo
    GraphVisualizer::print_adjacency_list(graph_data, 20);

    // Encontrar caminho entre robô e alvo
    if (graph_data.robot_index == -1 || graph_data.target_index == -1) {
        RCLCPP_ERROR(node->get_logger(), "Robô ou alvo não encontrado no mapa");
        rclcpp::shutdown();
        return 1;
    }

    auto path = PathFinder::find_shortest_path(
        graph_data,
        graph_data.robot_index,
        graph_data.target_index
    );

    if (path.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Não foi possível encontrar um caminho!");
        rclcpp::shutdown();
        return 1;
    }
    
    // Converter caminho para coordenadas e comandos
    auto coordinates = PathFinder::path_to_coordinates(graph_data, path);
    GraphVisualizer::print_path(coordinates);
    
    auto commands = coordinates_to_commands(coordinates);
    
    std::cout << "COMANDOS DE MOVIMENTO GERADOS" << std::endl;
    std::cout << "Total de comandos: " << commands.size() << std::endl;
    std::cout << "\nSequência de comandos:" << std::endl;
    for (size_t i = 0; i < commands.size(); i++) {
        std::cout << "  " << (i + 1) << ". " << commands[i] << std::endl;
    }
    
    std::cout << "Pressione ENTER para iniciar a execução dos movimentos" << std::endl;
    std::cin.get();
    
    bool success = execute_robot_movements(node, commands, 50);  // xms de delay

    if (success) {
        std::cout << "Todos os movimentos foram executados com sucesso!" << std::endl;
    } else {
        std::cout << "Houve falhas na execução dos movimentos." << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}