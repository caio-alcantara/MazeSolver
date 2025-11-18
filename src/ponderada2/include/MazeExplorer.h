#ifndef MAZE_EXPLORER_H
#define MAZE_EXPLORER_H

#include "RobotController.h"
#include <map>
#include <set>
#include <vector>

/**
 * @brief Posição no labirinto
 */
struct Position {
    int row;
    int col;
    
    bool operator<(const Position& other) const {
        if (row != other.row) return row < other.row;
        return col < other.col;
    }
    
    bool operator==(const Position& other) const {
        return row == other.row && col == other.col;
    }
};

/**
 * @brief Classe responsável pela exploração do labirinto
 */
class MazeExplorer {
public:
    /**
     * @brief Explora o labirinto usando DFS
     * @param node Nó ROS 2
     * @param sensor_sub Subscriber dos sensores
     * @param maze_map Mapa do labirinto (saída)
     * @param current_pos Posição atual do robô
     * @param target_pos Posição do alvo (saída)
     * @param target_found Flag indicando se o alvo foi encontrado (saída)
     * @param visited Conjunto de posições visitadas
     * @param move_history Histórico de movimentos
     */
    static void explore(
        rclcpp::Node::SharedPtr node,
        SensorSubscriber& sensor_sub,
        std::map<Position, SensorData>& maze_map,
        Position& current_pos,
        Position& target_pos,
        bool& target_found,
        std::set<Position>& visited,
        std::vector<std::string>& move_history
    );
};

#endif // MAZE_EXPLORER_H
