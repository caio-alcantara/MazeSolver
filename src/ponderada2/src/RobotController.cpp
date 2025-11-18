#include "RobotController.h"
#include <chrono>
#include <thread>
#include <iostream>

SensorSubscriber::SensorSubscriber(rclcpp::Node::SharedPtr node) 
    : node_(node), has_data_(false) {
    subscription_ = node_->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors",
        10,
        [this](const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
            latest_sensor_data_.up = (msg->up == "f" || msg->up == "t");
            latest_sensor_data_.down = (msg->down == "f" || msg->down == "t");
            latest_sensor_data_.left = (msg->left == "f" || msg->left == "t");
            latest_sensor_data_.right = (msg->right == "f" || msg->right == "t");
            latest_sensor_data_.up_left = (msg->up_left == "f" || msg->up_left == "t");
            latest_sensor_data_.up_right = (msg->up_right == "f" || msg->up_right == "t");
            latest_sensor_data_.down_left = (msg->down_left == "f" || msg->down_left == "t");
            latest_sensor_data_.down_right = (msg->down_right == "f" || msg->down_right == "t");
            
            latest_sensor_data_.target_up = (msg->up == "t");
            latest_sensor_data_.target_down = (msg->down == "t");
            latest_sensor_data_.target_left = (msg->left == "t");
            latest_sensor_data_.target_right = (msg->right == "t");
            
            has_data_ = true;
        }
    );
}

SensorData SensorSubscriber::get_sensor_data() {
    while (!has_data_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return latest_sensor_data_;
}

void SensorSubscriber::clear_data() {
    has_data_ = false;
}

bool RobotController::move(
    rclcpp::Node::SharedPtr node,
    const std::string& direction,
    SensorSubscriber& sensor_sub
) {
    auto move_client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    
    if (!move_client->wait_for_service(std::chrono::seconds(1))) {
        return false;
    }
    
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;
    
    sensor_sub.clear_data();
    
    auto future = move_client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return future.get()->success;
    }
    
    return false;
}

bool RobotController::execute_movements(
    rclcpp::Node::SharedPtr node,
    const std::vector<std::string>& commands,
    int delay_ms
) {
    auto move_client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    
    std::cout << "\nExecutando " << commands.size() << " movimentos..." << std::endl;
    
    while (!move_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) return false;
    }
    
    size_t successful_moves = 0;
    
    for (size_t i = 0; i < commands.size(); i++) {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = commands[i];
        
        auto future = move_client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->success) {
                successful_moves++;
            }
        }
        
        if (i < commands.size() - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    
    std::cout << "Movimentos executados: " << successful_moves << "/" << commands.size() << std::endl;
    
    return successful_moves == commands.size();
}

std::vector<std::string> RobotController::coordinates_to_commands(
    const std::vector<std::pair<int, int>>& coordinates
) {
    std::vector<std::string> commands;
    
    for (size_t i = 0; i < coordinates.size() - 1; i++) {
        int delta_row = coordinates[i + 1].first - coordinates[i].first;
        int delta_col = coordinates[i + 1].second - coordinates[i].second;
        
        if (delta_row == -1 && delta_col == 0) {
            commands.push_back("up");
        } else if (delta_row == 1 && delta_col == 0) {
            commands.push_back("down");
        } else if (delta_row == 0 && delta_col == -1) {
            commands.push_back("left");
        } else if (delta_row == 0 && delta_col == 1) {
            commands.push_back("right");
        }
    }
    
    return commands;
}
