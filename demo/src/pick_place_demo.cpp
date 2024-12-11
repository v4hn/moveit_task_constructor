/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task.h>

#include <moveit_msgs/DisplayTrajectory.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh, pnh("~");

	// Handle Task introspection requests from RViz & feedback during execution
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit_task_constructor_demo::setupDemoScene(pnh);

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		return 1;
	}

	if (pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
		if (pnh.param("execute", false)) {
			pick_place_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}

	ros::Publisher display_path_publisher =
	    ros::NodeHandle().advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	std::array<moveit_msgs::DisplayTrajectory, 2> d;
	auto& model = pick_place_task.task().getRobotModel();

	if (!pick_place_task.task().solutions().empty()) {
		ROS_INFO("extracting trajectories");
		for (size_t i = 0; i < 2; ++i) {
			d[i].model_id = model->getName();
			moveit::task_constructor::SolutionBaseConstPtr s{ *std::next(pick_place_task.task().solutions().begin(), i) };
			moveit_task_constructor_msgs::Solution solution_msg;
			s->appendTo(solution_msg);
			d[i].trajectory_start = solution_msg.start_scene.robot_state;
			// append all parts to trajectory
			ROS_INFO_STREAM("has " << solution_msg.sub_trajectory.size() << " sub trajectories");
			// we need a single element in d[i].trajectory, so we have to concatenate all sub trajectories
			robot_trajectory::RobotTrajectoryPtr t{ new robot_trajectory::RobotTrajectory(model) };
			t->setGroupName("panda_arm_hand");
			for (const auto& sub_trajectory : solution_msg.sub_trajectory) {
				auto& state{ (t->getWayPointCount() > 0) ? t->getLastWayPoint() : s->start()->scene()->getCurrentState() };
				robot_trajectory::RobotTrajectory rt{ model, "panda_arm_hand" };
				rt.setRobotTrajectoryMsg(state, sub_trajectory.trajectory);
				t->append(rt, 0.0);
			}
			d[i].trajectory.emplace_back();
			t->getRobotTrajectoryMsg(d[i].trajectory.back());
		}

		for (auto& display_trajectory : d) {
			for (auto& trajectory : display_trajectory.trajectory) {
				// Scale finger joint positions and velocities for visiblity
				int index = -1;
				for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); i++) {
					if (trajectory.joint_trajectory.joint_names[i] == "panda_finger_joint1") {
						index = i;
						break;
					}
				}
				if (index == -1) {
					ROS_ERROR_STREAM("Could not find panda_finger_joint1 in joint_names");
					return 1;
				}

				for (auto& joint_trajectory_point : trajectory.joint_trajectory.points) {
					joint_trajectory_point.positions[index] *= 15;
					joint_trajectory_point.velocities[index] *= 15;
				}
			}
		}

		ros::Rate r{ 0.5 };
		int i = 0;
		while (ros::ok()) {
			display_path_publisher.publish(d[i]);
			i = (i + 1) % 2;
			r.sleep();
		}
	}

	// If wanted, keep introspection alive
	if (pnh.param("keep_running", true)) {
		pick_place_task.introspection();
		ros::waitForShutdown();
	}

	return 0;
}
