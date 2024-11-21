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
   Desc:   Wrap multiple Cartesian motions with a reparametrization SerialContainer
*/

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/container.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#define M_TAU (2. * M_PI)

using namespace moveit::task_constructor;

void extractTrajectory(const SolutionBase& solution, robot_trajectory::RobotTrajectoryPtr& t) {
	// if(t->getWayPointCount() == 0) {
	// 	t->addSuffixWayPoint(solution.start()->scene()->getCurrentState(), 0.0);
	// }
	if (auto* sub = dynamic_cast<SubTrajectory const*>(&solution)) {
		if (sub->trajectory())
			t->append(*sub->trajectory(), 0.0);
	} else if (auto* sub = dynamic_cast<SolutionSequence const*>(&solution)) {
		for (const auto& s : sub->solutions())
			extractTrajectory(*s, t);
	} else if (auto* sub = dynamic_cast<WrappedSolution const*>(&solution)) {
		extractTrajectory(*sub->wrapped(), t);
	} else {
		throw std::runtime_error("encountered unknown solution type");
	}
}

class SmoothSerialWrapper : public WrapperBase
{
public:
	SmoothSerialWrapper(const std::string& name) : WrapperBase(name) {}

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		robot_model_ = robot_model;
		WrapperBase::init(robot_model);
	}

	void onNewSolution(const SolutionBase& s) override {
		liftSolution(s);  // testing

		// concatenate trajectories from subsolutions
		auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_);
		extractTrajectory(s, trajectory);
		trajectory->setGroupName("panda_arm");

		{
			trajectory_processing::TimeOptimalTrajectoryGeneration time_param{ 0.1, 0.1, 0.3 };
			if (!time_param.computeTimeStamps(*trajectory))
				throw std::runtime_error("time reparametrization failed");

			SubTrajectory sub_trajectory{ trajectory };
			sub_trajectory.setComment("smoother");
			sub_trajectory.setCost(s.cost());
			spawn(InterfaceState{ *s.start() }, InterfaceState{ *s.end() }, std::move(sub_trajectory));
		}
	}

private:
	robot_model::RobotModelConstPtr robot_model_;
};

Task createTask() {
	Task t;
	t.stages()->setName("Smooth Motions");

	auto cartesian = std::make_shared<solvers::CartesianPath>();

	auto c = std::make_unique<SerialContainer>("sequence");
	c->properties().set("group", "panda_arm");

	c->add(std::make_unique<stages::CurrentState>("current"));

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.2;
		stage->setDirection(direction);

		c->add(std::move(stage));
	}
	{
		auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setIKFrame([]() {
			geometry_msgs::PoseStamped f;
			f.header.frame_id = "panda_hand";
			f.pose.orientation.w = 1.0;
			f.pose.position.z = 0.1;
			return f;
		}());
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.x = -M_TAU / 8.;
		twist.twist.angular.y = -M_TAU / 8.;
		stage->setDirection(twist);
		c->add(std::move(stage));
	}

	// t.add(std::move(c));
	auto wrapper = std::make_unique<SmoothSerialWrapper>("smooth");
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
