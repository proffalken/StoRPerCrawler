cmake_minimum_required(VERSION 3.10)
project(storper_sim)

find_package(ament_cmake REQUIRED)

# Install launch, urdf, and world files
install(DIRECTORY
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install resource marker
install(FILES
  resource/storper_sim
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()

