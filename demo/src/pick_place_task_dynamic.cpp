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

#include <moveit_task_constructor_demo/pick_place_task_dynamic.h>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_demo");

namespace {
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
	return Eigen::Translation3d(values[0], values[1], values[2]) *
	       Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
	       Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}
}  // namespace

namespace moveit_task_constructor_demo {

PickPlaceTaskDynamic::PickPlaceTaskDynamic(const std::string& task_name, const std::string& ns)
  : task_name_(task_name), ns_(ns) {}

bool PickPlaceTaskDynamic::init(const rclcpp::Node::SharedPtr& node,
                                const pick_place_task_demo::Params& params,
                                const std::string& object_id,
                                const geometry_msgs::msg::PoseStamped& place_pose) {
	RCLCPP_INFO(LOGGER, "Initializing task pipeline for object '%s'", object_id.c_str());

	// Destroy the previous task before constructing the new one so that the
	// introspection node sends a reset message to RViz first.
	task_.reset();
	task_.reset(new moveit::task_constructor::Task(ns_));

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel(node);

	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(1.0);
	cartesian_planner->setMaxAccelerationScalingFactor(1.0);
	cartesian_planner->setStepSize(.01);

	t.setProperty("group", params.arm_group_name);
	t.setProperty("eef", params.eef_name);
	t.setProperty("hand", params.hand_group_name);
	t.setProperty("hand_grasping_frame", params.hand_frame);
	t.setProperty("ik_frame", params.hand_frame);

	// ── Current State ────────────────────────────────────────────────────────
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object_id](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object_id)) {
				comment = "object with id '" + object_id + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		t.add(std::move(applicability_filter));
	}

	// ── Open Hand ────────────────────────────────────────────────────────────
	Stage* initial_state_ptr = nullptr;
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup(params.hand_group_name);
		stage->setGoal(params.hand_open_pose);
		initial_state_ptr = stage.get();
		t.add(std::move(stage));
	}

	// ── Move to Pick ─────────────────────────────────────────────────────────
	{
		stages::Connect::GroupPlannerVector planners = { { params.arm_group_name, sampling_planner },
			                                              { params.hand_group_name, sampling_planner } };
		auto stage = std::make_unique<stages::Connect>("move to pick", planners);
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	// ── Pick Object ──────────────────────────────────────────────────────────
	Stage* pick_stage_ptr = nullptr;
	{
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

		// Approach
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", params.hand_frame);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.hand_frame;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		// Generate grasp pose — looks up object_id in the live planning scene
		{
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->setPreGraspPose(params.hand_open_pose);
			stage->setObject(object_id);
			stage->setAngleDelta(M_PI / 12);
			stage->setMonitoredStage(initial_state_ptr);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			grasp->insert(std::move(wrapper));
		}

		// Allow collision (hand, object)
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object_id,
			    t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			grasp->insert(std::move(stage));
		}

		// Close hand
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
			stage->setGroup(params.hand_group_name);
			stage->setGoal(params.hand_close_pose);
			grasp->insert(std::move(stage));
		}

		// Attach object
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object_id, params.hand_frame);
			grasp->insert(std::move(stage));
		}

		// Allow collision (object, support surface)
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object_id }, { params.surface_link }, true);
			grasp->insert(std::move(stage));
		}

		// Lift object
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist);
			stage->setIKFrame(params.hand_frame);
			stage->properties().set("marker_ns", "lift_object");
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.world_frame;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		// Forbid collision (object, support surface)
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object_id }, { params.surface_link }, false);
			grasp->insert(std::move(stage));
		}

		pick_stage_ptr = grasp.get();
		t.add(std::move(grasp));
	}

	// ── Move to Place ────────────────────────────────────────────────────────
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { params.arm_group_name, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	// ── Place Object ─────────────────────────────────────────────────────────
	{
		auto place = std::make_unique<SerialContainer>("place object");
		t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
		place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

		// Lower object
		{
			auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
			stage->properties().set("marker_ns", "lower_object");
			stage->properties().set("link", params.hand_frame);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.03, .13);
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.world_frame;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		// Generate place pose — use the PoseStamped from the incoming message
		{
			auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
			stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
			stage->properties().set("marker_ns", "place_pose");
			stage->setObject(object_id);

			// Apply the vertical offset so the object rests on the surface
			geometry_msgs::msg::PoseStamped target = place_pose;
			target.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
			stage->setPose(target);
			stage->setMonitoredStage(pick_stage_ptr);

			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(2);
			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		// Open hand
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
			stage->setGroup(params.hand_group_name);
			stage->setGoal(params.hand_open_pose);
			place->insert(std::move(stage));
		}

		// Forbid collision (hand, object)
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
			stage->allowCollisions(object_id, *t.getRobotModel()->getJointModelGroup(params.hand_group_name), false);
			place->insert(std::move(stage));
		}

		// Detach object
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object_id, params.hand_frame);
			place->insert(std::move(stage));
		}

		// Retreat after place
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.12, .25);
			stage->setIKFrame(params.hand_frame);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.hand_frame;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		t.add(std::move(place));
	}

	// ── Move to Home ─────────────────────────────────────────────────────────
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal(params.arm_home_pose);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}

	try {
		t.init();
	} catch (InitStageException& e) {
		RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
		return false;
	}

	return true;
}

bool PickPlaceTaskDynamic::plan(const std::size_t max_solutions) {
	RCLCPP_INFO(LOGGER, "Start searching for task solutions");
	return static_cast<bool>(task_->plan(max_solutions));
}

bool PickPlaceTaskDynamic::execute() {
	RCLCPP_INFO(LOGGER, "Executing solution trajectory");
	moveit_msgs::msg::MoveItErrorCodes result = task_->execute(*task_->solutions().front());
	if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
		RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << result.val);
		return false;
	}
	return true;
}

SolutionBaseConstPtr PickPlaceTaskDynamic::solution(uint32_t solution_id) const {
	if (!task_)
		return nullptr;
	for (const auto& s : task_->solutions()) {
		if (task_->introspection().solutionId(*s) == solution_id)
			return s;
	}
	return nullptr;
}

bool PickPlaceTaskDynamic::execute(uint32_t solution_id) {
	const SolutionBaseConstPtr s = solution(solution_id);
	if (!s) {
		RCLCPP_ERROR(LOGGER, "No solution with id %u", solution_id);
		return false;
	}
	RCLCPP_INFO(LOGGER, "Executing solution %u", solution_id);
	moveit_msgs::msg::MoveItErrorCodes result = task_->execute(*s);
	if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
		RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << result.val);
		return false;
	}
	return true;
}

}  // namespace moveit_task_constructor_demo
