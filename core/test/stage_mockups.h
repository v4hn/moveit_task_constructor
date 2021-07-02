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

	static PredefinedCosts constant(double c) { return PredefinedCosts{ { c }, false }; }

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

	PredefinedCosts costs_;
	size_t runs_{ 0 };

	static unsigned int id_;

	GeneratorMockup(PredefinedCosts&& costs = PredefinedCosts{ { 0.0 }, true });
	GeneratorMockup(std::list<double>&& costs) : GeneratorMockup{ PredefinedCosts{ std::move(costs), true } } {}

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	bool canCompute() const override;
	void compute() override;
};

// TODO(v4hn): migrate MonitoringGeneratorMockupup

struct ConnectMockup : public Connecting
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };

	static unsigned int id_;

	ConnectMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0));
	ConnectMockup(std::list<double>&& costs) : ConnectMockup{ PredefinedCosts{ std::move(costs), true } } {}

	using Connecting::compatible;
	void compute(const InterfaceState& from, const InterfaceState& to) override;
};

struct PropagatorMockup : public PropagatingEitherWay
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };
	std::size_t solutions_per_compute_;

	static unsigned int id_;

	PropagatorMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
	PropagatorMockup(std::list<double>&& costs, std::size_t solutions_per_compute = 1)
	  : PropagatorMockup{ PredefinedCosts{ std::move(costs), true }, solutions_per_compute } {}

	void computeForward(const InterfaceState& from) override;
	void computeBackward(const InterfaceState& to) override;
};

struct ForwardMockup : public PropagatorMockup
{
	planning_scene::PlanningScenePtr ps;
	static unsigned int id_;

	ForwardMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
	ForwardMockup(std::list<double>&& costs, std::size_t solutions_per_compute = 1)
	  : ForwardMockup{ PredefinedCosts{ std::move(costs), true }, solutions_per_compute } {}

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	void computeForward(const InterfaceState& from) override;
};

struct BackwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	BackwardMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
	BackwardMockup(std::list<double>&& costs, std::size_t solutions_per_compute = 1)
	  : BackwardMockup{ PredefinedCosts{ std::move(costs), true }, solutions_per_compute } {}
};

void resetMockupIds() {
	GeneratorMockup::id_ = 0;
	ConnectMockup::id_ = 0;
	ForwardMockup::id_ = 0;
	BackwardMockup::id_ = 0;
}

}  // namespace task_constructor
}  // namespace moveit
