cmake_minimum_required(VERSION 3.8)
project(kuka_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf resources meshes collisions launch
  DESTINATION share/${PROJECT_NAME}
)

# Create symlink to robotiq_85_description so Gazebo can find gripper meshes
install(CODE "
  set(SHARE_DIR \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
  set(ROBOTIQ_TARGET \"../../../robotiq_85_description/share/robotiq_85_description\")
  set(ROBOTIQ_LINK \"\${SHARE_DIR}/robotiq_85_description\")
  
  # Remove old symlink if it exists
  if(EXISTS \"\${ROBOTIQ_LINK}\")
    file(REMOVE \"\${ROBOTIQ_LINK}\")
  endif()
  
  # Create symlink
  execute_process(
    COMMAND \${CMAKE_COMMAND} -E create_symlink \${ROBOTIQ_TARGET} \${ROBOTIQ_LINK}
    RESULT_VARIABLE result
  )
  
  if(result)
    message(WARNING \"Failed to create symlink to robotiq_85_description\")
  else()
    message(STATUS \"Created symlink: \${ROBOTIQ_LINK} -> \${ROBOTIQ_TARGET}\")
  endif()
")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
