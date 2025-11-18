# 🎮 Culling Games - Guia Completo de Instalação e Execução (README GERADO POR IA)

## 📋 Sumário

1. [Objetivo do Projeto](#-objetivo-do-projeto)
2. [Pré-requisitos](#-pré-requisitos)
3. [Instalação](#-instalação)
4. [Estrutura do Projeto](#-estrutura-do-projeto)
5. [Como Executar](#-como-executar)
6. [Detalhamento Técnico](#-detalhamento-técnico)
7. [Troubleshooting](#-troubleshooting)

---

## 🎯 Objetivo do Projeto

Este projeto implementa dois algoritmos clássicos de busca em grafos aplicados à navegação robótica em labirintos:

### **Ponderada 1: BFS (Breadth-First Search)**
- Robô conhece o **mapa completo** desde o início
- Utiliza **BFS** para encontrar o caminho mais curto
- Acessa o serviço ROS `/get_map` para obter o labirinto
- Executa o caminho ótimo diretamente

### **Ponderada 2: DFS + BFS (Exploração e Otimização)**
- Robô **não conhece** o mapa inicialmente
- Usa **DFS com Backtracking** para explorar e mapear o labirinto
- Reconstrói o mapa baseado em sensores locais (`/robot_sensors`)
- Aplica **BFS** no mapa construído para encontrar o caminho ótimo
- Executa o caminho mais curto

**Objetivo Educacional:** Demonstrar a diferença entre busca com conhecimento completo (BFS puro) vs. exploração com conhecimento parcial (DFS + BFS).

---

## 📦 Pré-requisitos

### Sistema Operacional utilizado
- **Ubuntu 24.04 LTS** 
- Outras distribuições Linux podem funcionar, mas não são oficialmente suportadas

### Ferramentas Essenciais

#### 1. **Git**
```bash
sudo apt update
sudo apt install git -y
```

Verificar instalação:
```bash
git --version
# Exemplo de saída: git version 2.34.1
```

#### 2. **ROS 2 Jazzy**

**Instalação completa:**

```bash
# Adicionar repositório ROS 2
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Adicionar chave GPG
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Adicionar repositório à sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS 2 Humble Desktop
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y

# Instalar ferramentas de build
sudo apt install ros-dev-tools -y
```

**Configurar ambiente ROS 2:**

Adicione ao final do arquivo `~/.bashrc`:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verificar instalação:
```bash
ros2 --version
# Saída esperada: ros2 cli version humble
```

#### 3. **C++ Build Tools**

```bash
# Compilador G++
sudo apt install build-essential -y

# CMake
sudo apt install cmake -y

# Colcon (build tool do ROS 2)
sudo apt install python3-colcon-common-extensions -y
```

Verificar instalações:
```bash
g++ --version
# Saída esperada: g++ (Ubuntu 11.4.0-1ubuntu1~22.04) 11.4.0

cmake --version
# Saída esperada: cmake version 3.22.1

colcon version-check
# Saída esperada: colcon-argcomplete 0.3.x ...
```

#### 4. **Python 3 e venv**

```bash
# Python 3 (geralmente já vem instalado no Ubuntu 22.04)
sudo apt install python3 python3-pip python3-venv -y
```

Verificar instalação:
```bash
python3 --version
# Saída esperada: Python 3.10.x

pip3 --version
# Saída esperada: pip 22.x.x from ...
```

---

## 🚀 Instalação

### 1. Clonar o Repositório

```bash
# Navegue até o diretório onde deseja clonar o projeto
cd ~/

# Clone o repositório
git clone <URL_DO_REPOSITORIO> culling_games
cd culling_games
```

### 2. Configurar Ambiente Python (Virtual Environment)

```bash
# Criar ambiente virtual
python3 -m venv venv

# Ativar ambiente virtual
source venv/bin/activate

# Atualizar pip
pip install --upgrade pip

# Instalar dependências Python
pip install -r requirements.txt
```

**⚠️ IMPORTANTE:** O venv deve estar ativado sempre que você executar o projeto!

### 3. Compilar o Workspace ROS 2

```bash
# Certifique-se de estar no diretório raiz do projeto
cd ~/culling_games

# Compilar todos os pacotes
colcon build

# Se houver erros, tente compilar com mais detalhes:
# colcon build --event-handlers console_direct+
```

**Saída esperada:**
```
Starting >>> cg_interfaces
Finished <<< cg_interfaces [10.2s]
Starting >>> cg
Starting >>> cg_teleop
Starting >>> ponderada
Starting >>> ponderada2
Finished <<< cg [5.3s]
Finished <<< cg_teleop [3.1s]
Finished <<< ponderada [4.8s]
Finished <<< ponderada2 [5.2s]

Summary: 5 packages finished [15.7s]
```

### 4. Source do Workspace

```bash
# Source do workspace (necessário em CADA novo terminal)
source install/setup.bash
```

---

## 📁 Estrutura do Projeto

```
culling_games/
├── src/
│   ├── cg/                      # Pacote principal do jogo (Pygame)
│   │   ├── maps/                # Labirintos CSV
│   │   └── cg/                  # Nó ROS do jogo
│   ├── cg_interfaces/           # Mensagens e serviços customizados
│   ├── cg_teleop/               # Teleoperação por teclado
│   ├── ponderada/               # 🔵 PONDERADA 1: BFS
│   │   ├── include/             # Headers (.h)
│   │   └── src/                 # Implementação (.cpp)
│   │       ├── main.cpp         # Entry point
│   │       ├── PathFinder.cpp   # BFS
│   │       ├── GraphGenerator.cpp
│   │       └── ...
│   └── ponderada2/              # 🟢 PONDERADA 2: DFS + BFS
│       ├── include/             # Headers (.h)
│       └── src/                 # Implementação (.cpp)
│           ├── main.cpp         # Entry point
│           ├── MazeExplorer.cpp # DFS com Backtracking
│           ├── MapConverter.cpp # Converte sensores → grid
│           ├── PathFinder.cpp   # BFS
│           └── ...
├── build/                       # Arquivos de build (gerado)
├── install/                     # Executáveis instalados (gerado)
├── log/                         # Logs de compilação (gerado)
├── venv/                        # Ambiente virtual Python
├── requirements.txt             # Dependências Python
├── generate_maze.py             # Gerador de labirintos
└── README.md                    # Documentação original
```

---

## 🎮 Como Executar

### Passo 0: Preparação (TODO TERMINAL NOVO)

**Em CADA novo terminal, execute:**

```bash
# 1. Ativar venv
cd ~/culling_games
source venv/bin/activate

# 2. Source do ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Source do workspace
source install/setup.bash
```

---

### 🔵 Ponderada 1: BFS com Mapa Completo

#### **Terminal 1: Iniciar o Jogo**

```bash
# Iniciar o servidor do labirinto
ros2 run cg maze &
```

**O que acontece:**
- Janela Pygame abre mostrando o labirinto
- Robô (R) aparece na posição inicial
- Target (T) aparece na posição final
- Serviços ROS ficam disponíveis:
  - `/get_map` - Retorna o mapa completo
  - `/move_command` - Move o robô
  - `/reset` - Reinicia o jogo

#### **Terminal 2: Executar Ponderada 1**

```bash
# Executar algoritmo BFS
ros2 run ponderada main
```

**O que acontece:**

1. **Obtenção do Mapa:**
   ```
   [INFO] Solicitando mapa...
   [INFO] Mapa obtido: 29x29
   ```

2. **Geração do Grafo:**
   ```
   [INFO] Gerando grafo...
   [INFO] Vértices: 85
   [INFO] Robô índice: 0
   [INFO] Target índice: 84
   ```

3. **Busca BFS:**
   ```
   [INFO] Executando BFS...
   [INFO] Caminho encontrado! Tamanho: 23 movimentos
   ```

4. **Visualização:**
   ```
   Lista de Adjacência (primeiros 10 vértices):
   Vértice 0: 1(1)
   Vértice 1: 0(1) 2(1) 10(1)
   ...
   
   Caminho encontrado:
   (1,1) → (1,2) → (1,3) → ... → (13,13)
   Aperte enter para executar o movimento
   ```

5. **Execução:**
   ```
   [INFO] Executando movimentos...
   Movimento 1/23: right
   Movimento 2/23: right
   ...
   [INFO] Target alcançado! 🎯
   ```

**Na janela do jogo:** Você verá o robô se movendo automaticamente pelo caminho mais curto!

---

### 🟢 Ponderada 2: DFS + BFS com Exploração

#### **Terminal 1: Iniciar o Jogo**

```bash
# Iniciar o servidor do labirinto
ros2 run cg maze &
```

#### **Terminal 2: Executar Ponderada 2**

```bash
# Executar algoritmo DFS + BFS
ros2 run ponderada2 main
```

**O que acontece:**

1. **Fase de Exploração (DFS com Backtracking):**
   ```
   ========================================
   INICIANDO EXPLORAÇÃO DO LABIRINTO
   ========================================
   
   [EXPLORAÇÃO] Posição atual: (0, 0)
     Sensores: UP=B DOWN=B LEFT=B RIGHT=F
     [up] Bloqueado
     [right] Explorando...
   
   [EXPLORAÇÃO] Posição atual: (0, 1)
     Sensores: UP=B DOWN=F LEFT=F RIGHT=F
     [up] Bloqueado
     [right] Explorando...
   
   [EXPLORAÇÃO] Posição atual: (0, 2)
     >>> TARGET DETECTADO EM: (1, 2) [DOWN] <<<
     [down] Target detectado - não entrando
     
   [BACKTRACK] Voltando com left
   [EXPLORAÇÃO] Posição atual: (0, 1)
     [down] Explorando...
   ...
   
   ========================================
   EXPLORAÇÃO CONCLUÍDA
   ========================================
   Células exploradas: 85
   Target encontrado: Sim
   Posição do target: (13, 13)
   ```

2. **Conversão do Mapa:**
   ```
   ========================================
   CONVERTENDO MAPA EXPLORADO
   ========================================
   Dimensões: 15x15
   Células mapeadas: 85
   ```

3. **Geração do Grafo:**
   ```
   ========================================
   GERANDO GRAFO DO MAPA
   ========================================
   Vértices: 85
   Robô índice: 0
   Target índice: 84
   ```

4. **Busca BFS:**
   ```
   ========================================
   EXECUTANDO BFS
   ========================================
   Caminho encontrado! Tamanho: 23 movimentos
   
   Caminho:
   (1,1) → (1,2) → (1,3) → ... → (13,13)
   ```

5. **Retorno à Posição Inicial:**
   ```
   ========================================
   RETORNANDO À POSIÇÃO INICIAL
   ========================================
   Movimentos de retorno: 147
   Executando movimento 1/147: down
   Executando movimento 2/147: left
   ...
   [INFO] Robô retornou à posição inicial
   ```

6. **Execução do Caminho Ótimo:**
   ```
   ========================================
   EXECUTANDO CAMINHO ÓTIMO
   ========================================
   Executando movimento 1/23: right
   Executando movimento 2/23: right
   ...
   [INFO] Target alcançado! 🎯
   ```

**Na janela do jogo:** Você verá o robô explorando TODO o labirinto (vai e volta), depois retornando ao início e finalmente executando o caminho mais curto!

---

## 🔬 Detalhamento Técnico

### Algoritmos Utilizados

#### **BFS (Breadth-First Search)**
- **Arquivo:** `src/ponderada/src/PathFinder.cpp` e `src/ponderada2/src/PathFinder.cpp`
- **Complexidade:** O(V + E) onde V = vértices, E = arestas
- **Garantia:** Sempre encontra o caminho mais curto
- **Estrutura de dados:** Fila (FIFO)

#### **DFS com Backtracking**
- **Arquivo:** `src/ponderada2/src/MazeExplorer.cpp`
- **Complexidade:** O(V + E) no pior caso
- **Objetivo:** Exploração completa do espaço navegável
- **Característica:** Move fisicamente o robô, depois volta (backtracking)

#### **Geração do Grafo**
- **Arquivo:** `src/ponderada2/src/GraphGenerator.cpp`
- **Processo:**
  1. Mapeia células livres para índices de vértices
  2. Cria arestas entre células adjacentes (4 direções)
  3. Armazena em lista de adjacência (hashmap)

#### **Conversão de Mapa**
- **Arquivo:** `src/ponderada2/src/MapConverter.cpp`
- **Processo:**
  1. Recebe dados de sensores locais (`maze_map`)
  2. Calcula dimensões do mapa explorado
  3. Reconstrói grid 2D baseado em coordenadas relativas
  4. Infere células adjacentes usando dados dos sensores

### Estruturas de Dados Principais

```cpp
// Grafo - Lista de Adjacência com Hashmap
HashmapAdjacencyList {
    std::vector<std::unordered_map<int, int>> adj_map;
    // adj_map[u][v] = peso da aresta u→v
}

// Dados do Grafo
GraphData {
    HashmapAdjacencyList adj_list;
    std::map<std::pair<int,int>, int> pos_to_index;  // (linha,col) → índice
    std::map<int, std::pair<int,int>> index_to_pos;  // índice → (linha,col)
    int robot_index;
    int target_index;
}

// Posição Relativa (Ponderada 2)
struct Position {
    int row;
    int col;
};

// Dados dos Sensores (Ponderada 2)
struct SensorData {
    bool up, down, left, right;          // Células navegáveis
    bool up_left, up_right, down_left, down_right;  // Diagonais
    bool target_up, target_down, target_left, target_right;  // Target adjacente
};
```

---

## 🐛 Troubleshooting

### Problema 1: `colcon: command not found`

**Causa:** Colcon não está instalado ou não está no PATH.

**Solução:**
```bash
sudo apt install python3-colcon-common-extensions -y
source ~/.bashrc
```

---

### Problema 2: `Package 'cg_interfaces' not found`

**Causa:** Workspace não foi compilado ou sourced corretamente.

**Solução:**
```bash
cd ~/culling_games
colcon build
source install/setup.bash
```

---

### Problema 3: `error: pygame: No module named 'pygame'`

**Causa:** Dependências Python não foram instaladas ou venv não está ativado.

**Solução:**
```bash
source venv/bin/activate
pip install -r requirements.txt
```

---

### Problema 4: Robô não se move na Ponderada 1/2

**Causa:** Jogo não está rodando ou serviços não estão disponíveis.

**Verificar serviços:**
```bash
ros2 service list
```

Deve mostrar:
- `/get_map`
- `/move_command`
- `/reset`

**Solução:**
```bash
# Terminal 1
ros2 run cg maze &

# Aguardar janela abrir
sleep 2

# Terminal 2
ros2 run ponderada main  # ou ponderada2 main
```

---

### Problema 5: `CMake Error: Could not find a package configuration file`

**Causa:** ROS 2 não está sourced.

**Solução:**
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

### Problema 6: Janela Pygame fecha imediatamente

**Causa:** Venv não está ativado ou pygame não está instalado.

**Solução:**
```bash
source venv/bin/activate
pip list | grep pygame  # Verificar se está instalado
pip install pygame==2.5.2
```

---

### Problema 7: Build falha com erros de C++

**Limpar build e recompilar:**
```bash
rm -rf build/ install/ log/
colcon build
```

---
