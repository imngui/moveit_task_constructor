cmake_minimum_required(VERSION 3.16)
project(moveit_task_constructor_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME}_pick_place_task SHARED src/pick_place_task.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_task ${moveit_task_constructor_core_TARGETS})

include_directories(include)

# declare a demo consisting of a single cpp file
function(demo name)
	add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
	set(parameter_filename ${name}_parameters)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${parameter_filename}.yaml)
		generate_parameter_library(${parameter_filename} src/${parameter_filename}.yaml)
		target_link_libraries(${PROJECT_NAME}_${name} ${parameter_filename})
	endif()
	target_link_libraries(${PROJECT_NAME}_${name} ${moveit_task_constructor_core_TARGETS})
	set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
	install(TARGETS ${PROJECT_NAME}_${name}
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION lib/${PROJECT_NAME}
	)
endfunction()

demo(cartesian)
demo(modular)
demo(alternative_path_costs)
demo(ik_clearance_cost)
demo(fallbacks_move_to)

demo(pick_place_demo)
target_link_libraries(${PROJECT_NAME}_pick_place_task pick_place_demo_parameters)
target_link_libraries(${PROJECT_NAME}_pick_place_demo ${PROJECT_NAME}_pick_place_task)

# Dynamic pick-and-place: task class that accepts object_id and place_pose at runtime
add_library(${PROJECT_NAME}_pick_place_task_dynamic SHARED src/pick_place_task_dynamic.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_task_dynamic
	${moveit_task_constructor_core_TARGETS}
	pick_place_demo_parameters)
target_include_directories(${PROJECT_NAME}_pick_place_task_dynamic PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
# Exported so other packages (e.g. an action server) can link the dynamic task.
target_include_directories(${PROJECT_NAME}_pick_place_task PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
install(TARGETS ${PROJECT_NAME}_pick_place_task_dynamic ${PROJECT_NAME}_pick_place_task pick_place_demo_parameters
	EXPORT ${PROJECT_NAME}Targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
	INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

# Subscriber node: queues /pick_place_task messages and processes them serially
add_executable(${PROJECT_NAME}_pick_place_dynamic_demo src/pick_place_dynamic_demo.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_dynamic_demo
	${moveit_task_constructor_core_TARGETS}
	${moveit_task_constructor_msgs_TARGETS}
	${PROJECT_NAME}_pick_place_task
	${PROJECT_NAME}_pick_place_task_dynamic
	pick_place_demo_parameters)
set_target_properties(${PROJECT_NAME}_pick_place_dynamic_demo PROPERTIES
	OUTPUT_NAME pick_place_dynamic_demo PREFIX "")
install(TARGETS ${PROJECT_NAME}_pick_place_dynamic_demo
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/test_pick_place_dynamic.py
        scripts/alternatives.py
        scripts/cartesian.py
        scripts/compute_ik.py
        scripts/constrained.py
        scripts/current_state.py
        scripts/fallbacks.py
        scripts/fix_collision_objects.py
        scripts/fixed_state.py
        scripts/generate_pose.py
        scripts/merger.py
        scripts/modify_planning_scene.py
        scripts/multi_planner.py
        scripts/pickplace.py
        scripts/properties.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

# TODO: Port tests
# add_subdirectory(test)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_task_constructor_core moveit_task_constructor_msgs rclcpp generate_parameter_library)

ament_package()
