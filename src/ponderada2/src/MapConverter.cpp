#include "MapConverter.h"
#include <iostream>
#include <algorithm>

std::vector<std::vector<std::string>> MapConverter::to_grid(
    const std::map<Position, SensorData>& maze_map,
    const Position& robot_start,
    const Position& target_pos,
    bool target_found,
    int& min_row,
    int& min_col
) {
    // Encontra dimensões do mapa
    int max_row = robot_start.row;
    int max_col = robot_start.col;
    min_row = robot_start.row;
    min_col = robot_start.col;
    
    for (const auto& [pos, _] : maze_map) {
        min_row = std::min(min_row, pos.row);
        max_row = std::max(max_row, pos.row);
        min_col = std::min(min_col, pos.col);
        max_col = std::max(max_col, pos.col);
    }
    
    if (target_found) {
        min_row = std::min(min_row, target_pos.row);
        max_row = std::max(max_row, target_pos.row);
        min_col = std::min(min_col, target_pos.col);
        max_col = std::max(max_col, target_pos.col);
    }
    
    // Adiciona margem para paredes
    min_row -= 1;
    min_col -= 1;
    max_row += 1;
    max_col += 1;
    
    int rows = max_row - min_row + 1;
    int cols = max_col - min_col + 1;
    
    // Inicializa grid com paredes
    std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols, "b"));
    
    // Preenche células exploradas
    for (const auto& [pos, sensors] : maze_map) {
        int r = pos.row - min_row;
        int c = pos.col - min_col;
        
        grid[r][c] = (pos == robot_start) ? "r" : "f";
        
        if (sensors.up && r > 0 && grid[r-1][c] == "b") {
            grid[r-1][c] = sensors.target_up ? "t" : "f";
        }
        if (sensors.down && r < rows - 1 && grid[r+1][c] == "b") {
            grid[r+1][c] = sensors.target_down ? "t" : "f";
        }
        if (sensors.left && c > 0 && grid[r][c-1] == "b") {
            grid[r][c-1] = sensors.target_left ? "t" : "f";
        }
        if (sensors.right && c < cols - 1 && grid[r][c+1] == "b") {
            grid[r][c+1] = sensors.target_right ? "t" : "f";
        }
    }
    
    // Garante que o target está marcado
    if (target_found) {
        int target_r = target_pos.row - min_row;
        int target_c = target_pos.col - min_col;
        grid[target_r][target_c] = "t";
    }
    
    return grid;
}

void MapConverter::print(const std::vector<std::vector<std::string>>& grid) {
    std::cout << "MAPA EXPLORADO" << std::endl;
    
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            if (cell == "r") std::cout << "R ";
            else if (cell == "t") std::cout << "T ";
            else if (cell == "f") std::cout << ". ";
            else std::cout << "# ";
        }
        std::cout << std::endl;
    }
    
}
