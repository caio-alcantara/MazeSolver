#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include <string>
#include <vector>

/**
 * @brief Dados dos sensores do robô
 */
struct SensorData {
    bool up;
    bool down;
    bool left;
    bool right;
    bool up_left;
    bool up_right;
    bool down_left;
    bool down_right;
    bool target_up;
    bool target_down;
    bool target_left;
    bool target_right;
};

/**
 * @brief Subscriber para ler dados dos sensores do robô
 */
class SensorSubscriber {
public:
    explicit SensorSubscriber(rclcpp::Node::SharedPtr node);
    SensorData get_sensor_data();
    void clear_data();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr subscription_;
    SensorData latest_sensor_data_;
    bool has_data_;
};

/**
 * @brief Controlador para movimentação do robô
 */
class RobotController {
public:
    /**
     * @brief Move o robô em uma direção
     * @param node Nó ROS 2
     * @param direction Direção do movimento (up, down, left, right)
     * @param sensor_sub Subscriber dos sensores
     * @return true se o movimento foi bem-sucedido
     */
    static bool move(
        rclcpp::Node::SharedPtr node,
        const std::string& direction,
        SensorSubscriber& sensor_sub
    );

    /**
     * @brief Executa uma sequência de movimentos
     * @param node Nó ROS 2
     * @param commands Vetor de comandos de movimento
     * @param delay_ms Delay entre movimentos em milissegundos
     * @return true se todos os movimentos foram bem-sucedidos
     */
    static bool execute_movements(
        rclcpp::Node::SharedPtr node,
        const std::vector<std::string>& commands,
        int delay_ms = 300
    );

    /**
     * @brief Converte coordenadas em comandos de movimento
     * @param coordinates Vetor de pares (linha, coluna)
     * @return Vetor de comandos de direção
     */
    static std::vector<std::string> coordinates_to_commands(
        const std::vector<std::pair<int, int>>& coordinates
    );
};

#endif // ROBOT_CONTROLLER_H
