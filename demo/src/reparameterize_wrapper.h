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
	std::mutex mutex_;

public:
	ReparameterizeWrapper(const std::string& name, trajectory_processing::TimeParameterizationPtr reparameterize)
	  : WrapperBase{ name }, reparameterize_{ reparameterize } {
		properties().declare("publish_original", false, "republish original solution together with reparameterized one");
		properties().declare("recheck_validity", true, "recheck validity of reparameterized trajectory");
	}

	void setPublishOriginal(bool flag) { setProperty("publish_original", flag); }
	void setRecheckValidity(bool flag) { setProperty("recheck_validity", flag); }

	void onNewSolution(const moveit::task_constructor::SolutionBase& solution) override {
		std::lock_guard<std::mutex> lock(mutex_);

		if (properties().get<bool>("publish_original"))
			liftSolution(solution, solution.cost(), "original");

		bool recheck_validity = properties().get<bool>("recheck_validity");

		std::vector<moveit::task_constructor::SubTrajectory const*> seq;
		flattenSolution(solution, seq);

		new_states.emplace_back(*solution.start());
		moveit::task_constructor::InterfaceState& start{ new_states.back() };
		new_states.emplace_back(*solution.end());
		moveit::task_constructor::InterfaceState& end{ new_states.back() };
		moveit::task_constructor::InterfaceState* last;

		std::vector<const moveit::task_constructor::SolutionBase*> merged_seq;
		auto it = seq.cbegin();
		while (it != seq.cend()) {
			moveit::task_constructor::SubTrajectory s{};
			s.setStartState((it == seq.cbegin()) ? start : *last);

			auto e{ std::find_if(it, seq.cend(), [&](moveit::task_constructor::SubTrajectory const* s) -> bool {
				return !s->creator()->properties().hasProperty("ignore_for_reparameterization") &&
				       (!s->trajectory() || s->trajectory()->getGroup() != (*it)->trajectory()->getGroup());
			}) };

			if ((*it)->trajectory() && std::next(it) != e) {
				// [it,e) can be merged into a new trajectory
				auto t = std::make_shared<robot_trajectory::RobotTrajectory>(solution.start()->scene()->getRobotModel());
				t->setGroupName((*it)->trajectory()->getGroupName());

				double cost = 0.0;
				for (auto i = it; i != e; ++i) {
					cost += (*i)->cost();
					if (!(*i)->trajectory())
						continue;
					t->append(*(*i)->trajectory(), 0.0);
				}
				s.setCost(cost);

				if (!reparameterize_->computeTimeStamps(*t))
					throw std::runtime_error("time reparametrization failed");

				s.setTrajectory(t);

				if (recheck_validity && !s.start()->scene()->isPathValid(*t)) {
					s.markAsFailure("reparameterized trajectory became invalid");
				}

				for (auto i = it; i != e; ++i) {
					for (const auto& marker : (*i)->markers())
						s.markers().push_back(marker);
				}

				it = e;
			} else {
				s.setTrajectory((*it)->trajectory());
				s.setComment((*it)->comment());
				s.setCost((*it)->cost());
				s.markers() = (*it)->markers();
				++it;
			}

			s.setCreator(this);

			if (it != seq.cend()) {
				new_states.push_back(moveit::task_constructor::InterfaceState{ *(*std::prev(it))->end() });
				s.setEndState(new_states.back());
				last = &new_states.back();
			} else {
				s.setEndState(end);
			}

			new_solutions.push_back(std::move(s));
			merged_seq.push_back(&new_solutions.back());
		}

		auto sseq = std::make_shared<moveit::task_constructor::SolutionSequence>(std::move(merged_seq), solution.cost());
		sseq->setComment("smoother");
		spawn(moveit::task_constructor::InterfaceState{ start }, moveit::task_constructor::InterfaceState{ end },
		      std::move(sseq));
	}

private:
	trajectory_processing::TimeParameterizationPtr reparameterize_;

	std::list<moveit::task_constructor::SubTrajectory> new_solutions;
	std::list<moveit::task_constructor::InterfaceState> new_states;
};
