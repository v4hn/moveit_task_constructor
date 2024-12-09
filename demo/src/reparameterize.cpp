/*********************************************************************
 * Copyright (c) 2024, University of Hamburg
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

/* Author: Michael Goerner
   Desc:   Wrap multiple Cartesian motions with a reparametrization wrapper
*/

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/cost_terms.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "reparameterize_wrapper.h"

#define M_TAU (2. * M_PI)

using namespace moveit::task_constructor;

Task createTask() {
	Task t;
	t.stages()->setName("Smooth");

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setStepSize(0.05);

	auto c = std::make_unique<SerialContainer>("sequence");
	c->properties().set("group", "panda_arm");
	c->properties().set("tool", [] {
		geometry_msgs::PoseStamped f;
		f.header.frame_id = "panda_hand";
		f.pose.orientation.w = 1.0;
		f.pose.position.z = 0.1;
		return f;
	}());

	c->add(std::make_unique<stages::CurrentState>("current"));

	auto segment = [&](double y, double z, double r) {
		auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->properties().property("ik_frame").configureInitFrom(Stage::PARENT, "tool");
		geometry_msgs::TwistStamped direction;
		direction.header.frame_id = "world";
		direction.twist.linear.x = 0.0;
		direction.twist.linear.y = y;
		direction.twist.linear.z = z;
		direction.twist.angular.x = r;
		stage->setDirection(direction);
		c->add(std::move(stage));
	};

	segment(0.03, 0.2, M_TAU / 6);
	segment(0.05, -0.1, M_TAU / 6);
	segment(0.05, 0.1, -M_TAU / 6);
	segment(0.03, -0.2, -M_TAU / 6);

	auto tp = std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>(
	    /* path tolerance */ 0.5,
	    /* dt */ 0.1,
	    /* min angle change */ 0.001);
	auto wrapper = std::make_unique<ReparameterizeWrapper>("smooth", tp);
	wrapper->setCostTerm(std::make_shared<cost::TrajectoryDuration>());
	wrapper->setPublishOriginal(true);
	wrapper->setGroup("panda_arm");
	wrapper->add(std::move(c));

	t.add(std::move(wrapper));

	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	auto task = createTask();
	try {
		task.plan();
	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	ROS_INFO("done");

	if (task.solutions().empty()) {
		ROS_INFO("no solutions found");
		ros::waitForShutdown();
		return 1;
	}

	ros::Publisher display_path_publisher =
	    ros::NodeHandle().advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	std::array<moveit_msgs::DisplayTrajectory, 2> d;
	ROS_INFO("extracting trajectories");
	for (size_t i = 0; i < 2; ++i) {
		d[i].model_id = task.getRobotModel()->getName();
		SolutionBaseConstPtr s{ *std::next(task.solutions().begin(), i) };
		moveit_task_constructor_msgs::Solution solution_msg;
		s->appendTo(solution_msg);
		d[i].trajectory_start = solution_msg.start_scene.robot_state;
		// append all parts to trajectory
		ROS_INFO_STREAM("has " << solution_msg.sub_trajectory.size() << " sub trajectories");
		// we need a single element in d[i].trajectory, so we have to concatenate all sub trajectories
		robot_trajectory::RobotTrajectoryPtr t{ new robot_trajectory::RobotTrajectory(task.getRobotModel(),
			                                                                           "panda_arm") };
		t->setGroupName("panda_arm");
		for (const auto& sub_trajectory : solution_msg.sub_trajectory) {
			robot_trajectory::RobotTrajectory rt{ task.getRobotModel(), "panda_arm" };
			rt.setGroupName("panda_arm");
			rt.setRobotTrajectoryMsg(s->start()->scene()->getCurrentState(), sub_trajectory.trajectory);
			t->append(rt, 0.0);
		}
		d[i].trajectory.emplace_back();
		t->getRobotTrajectoryMsg(d[i].trajectory.back());
	}

	ros::Rate r{ 0.5 };
	int i = 0;
	while (ros::ok()) {
		display_path_publisher.publish(d[i]);
		i = (i + 1) % 2;
		r.sleep();
	}

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
