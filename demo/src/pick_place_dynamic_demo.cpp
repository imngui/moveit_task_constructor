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

/**
 * pick_place_dynamic_demo
 *
 * Subscribes to /pick_place_task (moveit_task_constructor_msgs/msg/PickPlaceTask).
 * Each message triggers a full pick-and-place cycle:
 *   1. Rebuild the MTC pipeline against the current planning scene
 *   2. Plan
 *   3. Execute
 *
 * Tasks are serialised in a queue — a new request received while a task is
 * running will be processed immediately afterwards.
 *
 * Robot/environment configuration (arm group, gripper, distances, …) is
 * loaded once at startup from the pick_place_demo_parameters YAML via the
 * standard ROS 2 parameter interface.
 *
 * Example usage (publish a task from the command line):
 *
 *   ros2 topic pub --once /pick_place_task \
 *     moveit_task_constructor_msgs/msg/PickPlaceTask \
 *     '{object_id: "object",
 *       place_pose: {header: {frame_id: "world"},
 *                    pose: {position: {x: 0.0, y: -0.3, z: 0.0},
 *                           orientation: {w: 1.0}}}}'
 */

#include <rclcpp/rclcpp.hpp>

#include <moveit_task_constructor_demo/pick_place_task.h>
#include <moveit_task_constructor_demo/pick_place_task_dynamic.h>
#include <moveit_task_constructor_msgs/msg/pick_place_task.hpp>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_dynamic_demo");

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto node = rclcpp::Node::make_shared("pick_place_dynamic_demo", node_options);

	// Load static robot/environment parameters from YAML
	const auto param_listener = std::make_shared<pick_place_task_demo::ParamListener>(node);
	const auto params = param_listener->get_params();

	// Spawn the initial scene (table + object) once
	moveit_task_constructor_demo::setupDemoScene(params);

	// Task queue shared between the ROS spinning thread and the main thread
	std::queue<moveit_task_constructor_msgs::msg::PickPlaceTask> task_queue;
	std::mutex queue_mutex;
	std::condition_variable queue_cv;

	// Subscriber: push incoming messages onto the queue and wake the main thread
	auto subscription = node->create_subscription<moveit_task_constructor_msgs::msg::PickPlaceTask>(
	    "/pick_place_task", rclcpp::QoS(10),
	    [&](const moveit_task_constructor_msgs::msg::PickPlaceTask::SharedPtr msg) {
		    RCLCPP_INFO(LOGGER, "Received task request for object '%s'", msg->object_id.c_str());
		    {
			    std::lock_guard<std::mutex> lock(queue_mutex);
			    task_queue.push(*msg);
		    }
		    queue_cv.notify_one();
	    });

	// Spin ROS callbacks in a background thread so the main thread can block
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	// Main thread: dequeue and process tasks one at a time
	moveit_task_constructor_demo::PickPlaceTaskDynamic pick_place_task("pick_place_task");

	while (rclcpp::ok()) {
		moveit_task_constructor_msgs::msg::PickPlaceTask task_msg;

		{
			std::unique_lock<std::mutex> lock(queue_mutex);
			queue_cv.wait(lock, [&] { return !task_queue.empty() || !rclcpp::ok(); });

			if (!rclcpp::ok())
				break;

			task_msg = task_queue.front();
			task_queue.pop();
		}

		RCLCPP_INFO(LOGGER, "Processing task: object='%s'", task_msg.object_id.c_str());

		if (!pick_place_task.init(node, params, task_msg.object_id, task_msg.place_pose)) {
			RCLCPP_ERROR(LOGGER, "Task initialisation failed — skipping");
			continue;
		}

		if (!pick_place_task.plan(params.max_solutions)) {
			RCLCPP_ERROR(LOGGER, "Planning failed — skipping execution");
			continue;
		}

		if (!pick_place_task.execute()) {
			RCLCPP_ERROR(LOGGER, "Execution failed");
		}
	}

	spinning_thread.join();
	return 0;
}
