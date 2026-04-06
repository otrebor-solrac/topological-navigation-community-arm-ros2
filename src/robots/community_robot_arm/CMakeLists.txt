cmake_minimum_required(VERSION 3.8)
project(community_robot_arm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch      
  meshes      
  urdf        
  rviz        
  scripts     
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
