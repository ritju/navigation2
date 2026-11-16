# Test dynamic parameters
ament_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
ament_target_dependencies(test_dynamic_parameters
  ${dependencies}
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
)

ament_add_gtest(test_occupied_goal_timeout test_occupied_goal_timeout.cpp)
ament_target_dependencies(test_occupied_goal_timeout ${dependencies})
target_link_libraries(test_occupied_goal_timeout ${library_name})
