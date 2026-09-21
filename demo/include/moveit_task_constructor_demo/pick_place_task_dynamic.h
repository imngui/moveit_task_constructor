/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_task_constructor_demo/pick_place_demo_parameters.hpp>

namespace moveit_task_constructor_demo {
using namespace moveit::task_constructor;

/**
 * Pick-and-place task that accepts the object id and place pose at planning
 * time rather than from a static parameter file.  All robot/environment
 * parameters (arm group, eef, distances, etc.) still come from the shared
 * pick_place_demo_parameters YAML and are passed as `static_params`.
 */
class PickPlaceTaskDynamic
{
public:
	/**
	 * @param task_name Name of the task's root stage
	 * @param ns        ROS namespace for introspection topics/services
	 *                  (e.g. "pick_place" -> /pick_place/description). Empty keeps
	 *                  the global topics that the default RViz config expects.
	 */
	PickPlaceTaskDynamic(const std::string& task_name, const std::string& ns = "");
	~PickPlaceTaskDynamic() = default;

	/**
	 * Build and initialise the MTC task pipeline.
	 *
	 * @param node          ROS 2 node for planner / robot-model loading
	 * @param static_params Robot/environment config (arm group, distances, …)
	 * @param object_id     Name of the collision object to pick (must already
	 *                      exist in the MoveIt planning scene)
	 * @param place_pose    Target pose for placing the object
	 */
	bool init(const rclcpp::Node::SharedPtr& node, const pick_place_task_demo::Params& static_params,
	          const std::string& object_id, const geometry_msgs::msg::PoseStamped& place_pose);

	bool plan(std::size_t max_solutions);

	/// Execute the best (front) solution.
	bool execute();

	/// Execute the solution with the given introspection id.
	bool execute(uint32_t solution_id);

	/// The underlying task, or nullptr before the first init(). Replaced on every init().
	moveit::task_constructor::TaskPtr task() const { return task_; }

	/// Find a top-level solution by its introspection id; nullptr if unknown.
	moveit::task_constructor::SolutionBaseConstPtr solution(uint32_t solution_id) const;

private:
	std::string task_name_;
	std::string ns_;
	moveit::task_constructor::TaskPtr task_;
};

}  // namespace moveit_task_constructor_demo
