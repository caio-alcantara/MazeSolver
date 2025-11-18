#ifndef MAP_CONVERTER_H
#define MAP_CONVERTER_H

#include "MazeExplorer.h"
#include "RobotController.h"
#include <vector>
#include <string>

/**
 * @brief Classe responsável por converter o mapa explorado em grid
 */
class MapConverter {
public:
    /**
     * @brief Converte o mapa explorado para formato de grid
     * @param maze_map Mapa explorado
     * @param robot_start Posição inicial do robô
     * @param target_pos Posição do alvo
     * @param target_found Se o alvo foi encontrado
     * @param min_row Offset de linha (saída)
     * @param min_col Offset de coluna (saída)
     * @return Grid 2D representando o mapa
     */
    static std::vector<std::vector<std::string>> to_grid(
        const std::map<Position, SensorData>& maze_map,
        const Position& robot_start,
        const Position& target_pos,
        bool target_found,
        int& min_row,
        int& min_col
    );

    /**
     * @brief Imprime o mapa explorado
     * @param grid Grid 2D do mapa
     */
    static void print(const std::vector<std::vector<std::string>>& grid);
};

#endif // MAP_CONVERTER_H
