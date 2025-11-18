#include "MazeExplorer.h"

void MazeExplorer::explore(
    rclcpp::Node::SharedPtr node,
    SensorSubscriber& sensor_sub,
    std::map<Position, SensorData>& maze_map,
    Position& current_pos,
    Position& target_pos,
    bool& target_found,
    std::set<Position>& visited,
    std::vector<std::string>& move_history
) {
    visited.insert(current_pos);
    
    SensorData sensors = sensor_sub.get_sensor_data();
    maze_map[current_pos] = sensors;
    
    // Verifica se o target está adjacente
    if (sensors.target_up) {
        target_pos = {current_pos.row - 1, current_pos.col};
        target_found = true;
        visited.insert(target_pos);
    }
    if (sensors.target_down) {
        target_pos = {current_pos.row + 1, current_pos.col};
        target_found = true;
        visited.insert(target_pos);
    }
    if (sensors.target_left) {
        target_pos = {current_pos.row, current_pos.col - 1};
        target_found = true;
        visited.insert(target_pos);
    }
    if (sensors.target_right) {
        target_pos = {current_pos.row, current_pos.col + 1};
        target_found = true;
        visited.insert(target_pos);
    }
    
    // Define direções de exploração
    std::vector<std::pair<std::string, Position>> directions = {
        {"up", {current_pos.row - 1, current_pos.col}},
        {"right", {current_pos.row, current_pos.col + 1}},
        {"down", {current_pos.row + 1, current_pos.col}},
        {"left", {current_pos.row, current_pos.col - 1}}
    };
    
    std::vector<bool> can_move = {
        sensors.up && !sensors.target_up,
        sensors.right && !sensors.target_right,
        sensors.down && !sensors.target_down,
        sensors.left && !sensors.target_left
    };
    
    for (size_t i = 0; i < directions.size(); i++) {
        if (!can_move[i]) continue;
        
        Position next_pos = directions[i].second;
        
        if (visited.count(next_pos)) continue;
        
        if (RobotController::move(node, directions[i].first, sensor_sub)) {
            Position old_pos = current_pos;
            current_pos = next_pos;
            move_history.push_back(directions[i].first);
            
            explore(node, sensor_sub, maze_map, current_pos, target_pos, target_found, visited, move_history);
            
            // Backtrack
            std::string opposite;
            if (directions[i].first == "up") opposite = "down";
            else if (directions[i].first == "down") opposite = "up";
            else if (directions[i].first == "left") opposite = "right";
            else if (directions[i].first == "right") opposite = "left";
            
            if (RobotController::move(node, opposite, sensor_sub)) {
                current_pos = old_pos;
                move_history.pop_back();
            }
        }
    }
}
