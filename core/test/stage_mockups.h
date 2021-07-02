#pragma once

#include <moveit/task_constructor/stage_p.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit {
namespace task_constructor {

MOVEIT_STRUCT_FORWARD(PredefinedCosts);
struct PredefinedCosts : CostTerm
{
	mutable std::list<double> costs_;  // list of costs to assign
	mutable bool finite_;

	PredefinedCosts(std::list<double>&& costs, bool finite = true);
	PredefinedCosts(std::initializer_list<double> costs, bool finite = true)
	  : PredefinedCosts{ std::list<double>{ costs }, finite } {};

	static PredefinedCosts constant(double c) { return PredefinedCosts{ std::list<double>{ c }, false }; }

	bool exhausted() const;
	double cost() const;

	double operator()(const SubTrajectory& /*s*/, std::string& /*comment*/) const override { return cost(); }
	double operator()(const SolutionSequence& /*s*/, std::string& /*comment*/) const override { return cost(); }
	double operator()(const WrappedSolution& /*s*/, std::string& /*comment*/) const override { return cost(); }
};

struct GeneratorMockup : public Generator
{
	planning_scene::PlanningScenePtr ps_;

	PredefinedCosts costs_;
	size_t runs_{ 0 };

	static unsigned int id_;

	// default to one solution to avoid infinity loops
	GeneratorMockup(PredefinedCosts&& costs = PredefinedCosts{ std::list<double>{ 0.0 }, true });

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	bool canCompute() const override;
	void compute() override;
};

struct MonitoringGeneratorMockup : public MonitoringGenerator
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };

	static unsigned int id_;

	MonitoringGeneratorMockup(Stage* monitored, PredefinedCosts&& costs = PredefinedCosts::constant(0.0));

	bool canCompute() const override { return false; }
	void compute() override {}
	void onNewSolution(const SolutionBase& s) override;
};

struct ConnectMockup : public Connecting
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };

	static unsigned int id_;

	ConnectMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0));

	using Connecting::compatible;  // make this accessible for testing

	void compute(const InterfaceState& from, const InterfaceState& to) override;
};

struct PropagatorMockup : public PropagatingEitherWay
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };
	std::size_t solutions_per_compute_;

	static unsigned int id_;

	PropagatorMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);

	void computeForward(const InterfaceState& from) override;
	void computeBackward(const InterfaceState& to) override;
};

struct ForwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	ForwardMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
};

struct BackwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	BackwardMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
};

void resetMockupIds() {
	GeneratorMockup::id_ = 0;
	MonitoringGeneratorMockup::id_ = 0;
	ConnectMockup::id_ = 0;
	ForwardMockup::id_ = 0;
	BackwardMockup::id_ = 0;
}

}  // namespace task_constructor
}  // namespace moveit
