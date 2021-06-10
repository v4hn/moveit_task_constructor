#include <moveit/task_constructor/stage_p.h>
#include <moveit/planning_scene/planning_scene.h>

#include "mockups.h"

namespace moveit {
namespace task_constructor {

static unsigned int MOCK_ID = 0;
const double TRAJECTORY_DURATION{ 9.0 };

unsigned int GeneratorMockup::id_ = 0;
unsigned int ForwardMockup::id_ = 0;
unsigned int BackwardMockup::id_ = 0;
unsigned int ConnectMockup::id_ = 0;

PredefinedCosts::PredefinedCosts(bool finite, std::list<double>&& costs) : costs_(std::move(costs)), finite_(finite) {}

bool PredefinedCosts::exhausted() const {
	return finite_ && costs_.empty();
}

double PredefinedCosts::cost() const {
	if (!costs_.empty()) {
		cost_ = costs_.front();
		costs_.pop_front();
	}
	return cost_;
}

// If costs are used, set runs_=-1
GeneratorMockup::GeneratorMockup(std::initializer_list<double> costs)
  : Generator("GEN" + std::to_string(++id_)), costs_(true, costs), runs_(-1) {
	prev_.reset(new Interface);
	next_.reset(new Interface);
	pimpl()->setPrevEnds(prev_);
	pimpl()->setNextStarts(next_);
}

// If runs are used, set costs_ to be infinite
GeneratorMockup::GeneratorMockup(int runs)
  : Generator("GEN" + std::to_string(++id_)), costs_(false, { 0.0 }), runs_(runs) {
	prev_.reset(new Interface);
	next_.reset(new Interface);
	pimpl()->setPrevEnds(prev_);
	pimpl()->setNextStarts(next_);
}

void GeneratorMockup::init(const moveit::core::RobotModelConstPtr& robot_model) {
	ps_.reset(new planning_scene::PlanningScene(robot_model));
	Generator::init(robot_model);
}

bool GeneratorMockup::canCompute() const {
	// check if runs are being used and if there are still runs left (costs are then never exhausted) or if costs
	// are being used and they are not exhausted yet
	return runs_ > 0 || ((runs_ == -1) && !costs_.exhausted());
}

void GeneratorMockup::compute() {
	if (runs_ > 0) {
		--runs_;
	}
	spawn(InterfaceState(ps_), costs_.cost());
}

ConnectMockup::ConnectMockup(std::initializer_list<double> costs)
  : Connecting("CON" + std::to_string(++id_)), costs_(true, costs), runs_(-1) {}
ConnectMockup::ConnectMockup(int runs)
  : Connecting("CON" + std::to_string(++id_)), costs_(false, { 0.0 }), runs_(runs) {}

void ConnectMockup::compute(const InterfaceState& from, const InterfaceState& to) {
	if (runs_ == 0) {
		return;
	} else if (runs_ > 0) {
		--runs_;
	}
	auto solution{ std::make_shared<SubTrajectory>() };
	solution->setCost(costs_.cost());
	connect(from, to, solution);
}

PropagatorMockup::PropagatorMockup(std::initializer_list<double> costs, std::size_t solutions_per_compute)
  : PropagatingEitherWay()
  , fw_runs_(-1)
  , bw_runs_(-1)
  , costs_(false, costs)
  , solutions_per_compute_(solutions_per_compute) {}

PropagatorMockup::PropagatorMockup(int fw_runs, int bw_runs)
  : PropagatingEitherWay()
  , fw_runs_(fw_runs)
  , bw_runs_(bw_runs)
  , costs_(false, { 0.0 })
  , solutions_per_compute_(1) {}

void PropagatorMockup::computeForward(const InterfaceState& from) {
	++calls_;
	if (fw_runs_ == 0) {
		return;
	} else if (fw_runs_ > 0) {
		--fw_runs_;
	}
	for (std::size_t i = 0; i < solutions_per_compute_; ++i) {
		SubTrajectory solution(robot_trajectory::RobotTrajectoryConstPtr(), costs_.cost());
		sendForward(from, InterfaceState(from.scene()->diff()), std::move(solution));
	}
}

void PropagatorMockup::computeBackward(const InterfaceState& to) {
	++calls_;
	if (bw_runs_ == 0) {
		return;
	} else if (bw_runs_ > 0) {
		--bw_runs_;
	}
	for (std::size_t i = 0; i < solutions_per_compute_; ++i) {
		SubTrajectory solution(robot_trajectory::RobotTrajectoryConstPtr(), costs_.cost());
		sendBackward(InterfaceState(to.scene()->diff()), to, std::move(solution));
	}
}

ForwardMockup::ForwardMockup(std::initializer_list<double> costs, std::size_t solutions_per_compute)
  : PropagatorMockup(costs, solutions_per_compute) {
	restrictDirection(FORWARD);
	setName("FW" + std::to_string(++id_));
}

ForwardMockup::ForwardMockup(int runs) : PropagatorMockup(runs, 0) {
	restrictDirection(FORWARD);
	setName("FW" + std::to_string(++id_));
}

void ForwardMockup::init(const moveit::core::RobotModelConstPtr& robot_model) {
	ps.reset(new planning_scene::PlanningScene(robot_model));
	PropagatorMockup::init(robot_model);
}

void ForwardMockup::computeForward(const InterfaceState& from) {
	SubTrajectory solution;
	auto traj{ std::make_shared<robot_trajectory::RobotTrajectory>(ps->getRobotModel(),
		                                                            ps->getRobotModel()->getJointModelGroups()[0]) };
	traj->addSuffixWayPoint(ps->getCurrentState(), 0.0);
	traj->addSuffixWayPoint(ps->getCurrentState(), TRAJECTORY_DURATION);
	solution.setTrajectory(traj);
	solution.setCost(costs_.cost());
	InterfaceState to(from);

	sendForward(from, std::move(to), std::move(solution));
}

BackwardMockup::BackwardMockup(std::initializer_list<double> costs) : PropagatorMockup(costs) {
	restrictDirection(BACKWARD);
	setName("BW" + std::to_string(++id_));
}

BackwardMockup::BackwardMockup(int runs) : PropagatorMockup(0, runs) {
	restrictDirection(BACKWARD);
	setName("BW" + std::to_string(++id_));
}

}  // namespace task_constructor
}  // namespace moveit
