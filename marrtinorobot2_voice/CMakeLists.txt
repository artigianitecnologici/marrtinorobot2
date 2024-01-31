cmake_minimum_required(VERSION 3.5)
project(tts_node)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Install Python modules
install(
  DIRECTORY tts_node
  DESTINATION ${PYTHON_INSTALL_DIR}
)

ament_package()
