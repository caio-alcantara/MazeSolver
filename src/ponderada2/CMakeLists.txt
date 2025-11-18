cmake_minimum_required(VERSION 3.8)
project(ponderada2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# Adiciona o diretório de includes
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Adiciona o executável com todos os arquivos fonte
add_executable(main 
  src/main.cpp
  src/HashmapAdjacencyList.cpp
  src/Utils.cpp
  src/GraphGenerator.cpp
  src/RosMapClient.cpp
  src/PathFinder.cpp
  src/GraphVisualizer.cpp
  src/RobotController.cpp
  src/MazeExplorer.cpp
  src/MapConverter.cpp
)

ament_target_dependencies(main rclcpp cg_interfaces) # Dependências
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
