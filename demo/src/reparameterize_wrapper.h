#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/container.h>
#include <moveit/trajectory_processing/time_parameterization.h>

void flattenSolution(const moveit::task_constructor::SolutionBase& solution,
                     std::vector<const moveit::task_constructor::SubTrajectory*>& seq) {
	if (auto* sub = dynamic_cast<moveit::task_constructor::SubTrajectory const*>(&solution)) {
		seq.push_back(sub);
	} else if (auto* sub = dynamic_cast<moveit::task_constructor::SolutionSequence const*>(&solution)) {
		for (const auto& s : sub->solutions())
			flattenSolution(*s, seq);
	} else if (auto* sub = dynamic_cast<moveit::task_constructor::WrappedSolution const*>(&solution)) {
		flattenSolution(*sub->wrapped(), seq);
	} else {
		throw std::runtime_error("encountered unknown solution type");
	}
}

class ReparameterizeWrapper : public moveit::task_constructor::WrapperBase
{
public:
	ReparameterizeWrapper(const std::string& name, trajectory_processing::TimeParameterizationPtr reparameterize)
	  : WrapperBase{ name }, reparameterize_{ reparameterize } {
		properties().declare("publish_original", false, "republish original solution together with reparameterized one");
		properties().declare<std::string>("group", "joint model group to reparameterize");
	}

	void setPublishOriginal(bool flag) { setProperty("publish_original", flag); }
	void setGroup(std::string group) { setProperty("group", group); }

	void onNewSolution(const moveit::task_constructor::SolutionBase& solution) override {
		if (properties().get<bool>("publish_original"))
			liftSolution(solution, solution.cost(), "unchanged ");

		std::vector<const moveit::task_constructor::SubTrajectory*> seq;
		flattenSolution(solution, seq);

		std::vector<const moveit::task_constructor::SolutionBase*> merged_seq;
		auto it = seq.cbegin();
		while (it != seq.cend()) {
			auto e{ std::find_if(it, seq.cend(), [](auto& s) -> bool { return !s->trajectory(); }) };
			if (it == e) {
				merged_seq.push_back(*it);
				it = std::next(e);
			} else {
				// [it,end) should be merged into a new trajectory
				auto t = std::make_shared<robot_trajectory::RobotTrajectory>(solution.start()->scene()->getRobotModel());
				t->setGroupName(properties().get<std::string>("group"));

				double cost = 0.0;
				for (auto i = it; i != e; ++i) {
					t->append(*(*i)->trajectory(), 0.0);
					cost += (*i)->cost();
				}

				if (!reparameterize_->computeTimeStamps(*t))
					throw std::runtime_error("time reparametrization failed");

				moveit::task_constructor::SubTrajectory s{ t };
				s.setStartState(*(*it)->start());
				s.setEndState(*(*std::prev(e))->end());
				s.setCost(cost);

				merged_solutions.push_back(std::move(s));
				merged_seq.push_back(&merged_solutions.back());

				it = e;
			}
		}

		auto sseq = std::make_shared<moveit::task_constructor::SolutionSequence>(std::move(merged_seq), solution.cost());
		sseq->setComment("smoother");
		spawn(moveit::task_constructor::InterfaceState{ *solution.start() },
		      moveit::task_constructor::InterfaceState{ *solution.end() }, std::move(sseq));
	}

private:
	trajectory_processing::TimeParameterizationPtr reparameterize_;

	std::list<moveit::task_constructor::SubTrajectory> merged_solutions;
};
