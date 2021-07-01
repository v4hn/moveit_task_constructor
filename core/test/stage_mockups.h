#pragma once

#include <moveit/task_constructor/stage_p.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit {
namespace task_constructor {

MOVEIT_STRUCT_FORWARD(PredefinedCosts);
struct PredefinedCosts : CostTerm
{
	mutable std::list<double> costs_;  // list of costs to assign
	mutable double cost_ = 0.0;  // last assigned cost
	bool finite_;  // finite number of compute() attempts?

	PredefinedCosts(bool finite, std::list<double>&& costs);
	bool exhausted() const;
	double cost() const;

	double operator()(const SubTrajectory& /*s*/, std::string& /*comment*/) const override { return cost(); }
	double operator()(const SolutionSequence& /*s*/, std::string& /*comment*/) const override { return cost(); }
	double operator()(const WrappedSolution& /*s*/, std::string& /*comment*/) const override { return cost(); }
};

struct GeneratorMockup : public Generator
{
public:
	planning_scene::PlanningScenePtr ps_;
	InterfacePtr prev_;
	InterfacePtr next_;

	PredefinedCosts costs_;
	int runs_;
	static unsigned int id_;

	GeneratorMockup(std::initializer_list<double> costs = { 0.0 });
	GeneratorMockup(int runs);
	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	bool canCompute() const override;
	void compute() override;
};

struct ConnectMockup : public Connecting
{
	PredefinedCosts costs_;
	int runs_;
	static unsigned int id_;

	ConnectMockup(std::initializer_list<double> costs = { 0.0 });
	ConnectMockup(int runs);
	using Connecting::compatible;
	void compute(const InterfaceState& from, const InterfaceState& to) override;
};

struct PropagatorMockup : public PropagatingEitherWay
{
	PredefinedCosts costs_;
	int fw_runs_;
	int bw_runs_;
	std::size_t solutions_per_compute_;
	unsigned int calls_ = 0;

	PropagatorMockup(std::initializer_list<double> costs = { 0.0 }, std::size_t solutions_per_compute = 1);
	PropagatorMockup(int fw_runs, int bw_runs);
	void computeForward(const InterfaceState& from) override;
	void computeBackward(const InterfaceState& to) override;
};

struct ForwardMockup : public PropagatorMockup
{
	planning_scene::PlanningScenePtr ps;
	static unsigned int id_;

	ForwardMockup(std::initializer_list<double> costs = { 0.0 }, std::size_t solutions_per_compute = 1);
	ForwardMockup(int runs);
	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	void computeForward(const InterfaceState& from) override;
};

struct BackwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	BackwardMockup(std::initializer_list<double> costs = { 0.0 });
	BackwardMockup(int runs);
};

void resetIds() {
	GeneratorMockup::id_ = 0;
	ConnectMockup::id_ = 0;
	ForwardMockup::id_ = 0;
	BackwardMockup::id_ = 0;
}

}  // namespace task_constructor
}  // namespace moveit
