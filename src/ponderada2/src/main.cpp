#include "rclcpp/rclcpp.hpp"
#include "GraphGenerator.h"
#include "PathFinder.h"
#include "RobotController.h"
#include "MazeExplorer.h"
#include "MapConverter.h"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("main");
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "INICIANDO EXPLORAÇÃO DO LABIRINTO" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Inicializa subscriber dos sensores
    SensorSubscriber sensor_sub(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Estruturas para exploração
    std::map<Position, SensorData> maze_map;
    Position robot_start = {0, 0};
    Position current_pos = robot_start;
    Position target_pos = {0, 0};
    bool target_found = false;
    std::set<Position> visited;
    std::vector<std::string> move_history;
    
    // Fase 1: Exploração
    std::cout << "FASE 1: EXPLORAÇÃO" << std::endl;
    MazeExplorer::explore(node, sensor_sub, maze_map, current_pos, target_pos, 
                          target_found, visited, move_history);
    
    std::cout << "\nCélulas exploradas: " << maze_map.size() << std::endl;
    
    if (!target_found) {
        std::cerr << "Erro: Alvo não encontrado durante a exploração" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    std::cout << "Alvo encontrado em: (" << target_pos.row << ", " << target_pos.col << ")" << std::endl;
    
    // Converte mapa explorado para grid
    int min_row, min_col;
    auto grid = MapConverter::to_grid(maze_map, robot_start, target_pos, target_found, min_row, min_col);
    MapConverter::print(grid);
    
    // Fase 2: Pathfinding
    std::cout << "\nFASE 2: PATHFINDING" << std::endl;
    auto graph_data = gerar_lista_de_adjacencia(grid);
    
    if (graph_data.robot_index == -1 || graph_data.target_index == -1) {
        std::cerr << "Erro: Robô ou alvo não encontrado no grafo" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    auto path = PathFinder::find_shortest_path(
        graph_data,
        graph_data.robot_index,
        graph_data.target_index
    );

    if (path.empty()) {
        std::cerr << "Erro: Nenhum caminho encontrado" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    auto coordinates = PathFinder::path_to_coordinates(graph_data, path);
    auto commands = RobotController::coordinates_to_commands(coordinates);
    
    std::cout << "Caminho encontrado: " << commands.size() << " movimentos" << std::endl;
    
    // Fase 3: Execução
    std::cout << "\nPressione ENTER para executar..." << std::endl;
    std::cin.get();
    
    bool success = RobotController::execute_movements(node, commands, 100);

    if (success) {
        std::cout << "\n✓ MISSÃO CONCLUÍDA!" << std::endl;
    } else {
        std::cout << "\n⚠ Houve falhas na execução" << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
