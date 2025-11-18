#include "Utils.h"
#include <algorithm>
#include <iostream>

std::string to_lower(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

std::vector<std::vector<std::string>> converter_para_matriz_2d(
    const std::vector<std::string> &flat,
    int altura,
    int largura
) {
    std::vector<std::vector<std::string>> matriz;

    // Validação: o tamanho do array deve ser altura × largura
    if (flat.size() != static_cast<size_t>(altura * largura)) {
        std::cerr << "Erro: tamanho do array não corresponde às dimensões fornecidas." << std::endl;
        return matriz;
    }

    // Reconstrói a matriz linha por linha
    for (int i = 0; i < altura; i++) {
        std::vector<std::string> linha;
        for (int j = 0; j < largura; j++) {
            int indice = i * largura + j;
            linha.push_back(flat[indice]);
        }
        matriz.push_back(linha);
    }

    return matriz;
}

void imprimir_mapa(
    const std::vector<std::vector<std::string>> &matriz,
    const rclcpp::Logger &logger
) {
    if (matriz.empty()) {
        RCLCPP_ERROR(logger, "A matriz está vazia, nada para imprimir.");
        return;
    }

    RCLCPP_INFO(logger, "Mapa do labirinto:");
    
    for (const auto &linha : matriz) {
        std::string linha_str;
        for (const auto &celula : linha) {
            linha_str += celula + " ";
        }
        RCLCPP_INFO(logger, "%s", linha_str.c_str());
    }
}
