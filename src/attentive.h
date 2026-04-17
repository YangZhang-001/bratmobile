#ifndef ATTENTIVE_H
#define ATTENTIVE_H
#include "focused.h"

/**
 * @brief Configurator with long-range planning. It explores transitions out of a state until a DEFAULT Task is reached.
 * The next state to expand will be the one with lowest heuristic cost. 
 * 
 */
class AttentiveConfigurator: public virtual FocusedConfigurator{
	protected:

	/**
	 * Guard Psi: does not limit expansion to current vertex
	 */
	virtual bool preventTransition(vertexDescriptor v)override{
		return false;
	}

	/**
	* @brief Find a suitable sequence for execution as a plan in the transition system, if present
	*/
	bool recycle_plan(vertexDescriptor v, vertexDescriptor &v0, vertexDescriptor & task_start, StateMatcher::MATCH_TYPE &matchType, 
					b2Transform & shift_start, b2Transform& sk_first_start, std::pair<edgeDescriptor, bool>&edge,
					std::vector<vertexDescriptor> &plan_prov, Direction t_get_direction)override;

	virtual VertexMatch findMatch(State s, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, StateDifference * _sd=NULL, vertexDescriptor src=TransitionSystem::null_vertex())override;

	std::pair<bool, vertexDescriptor> isPlannedTaskOK(vertexDescriptor src, State s);


};

/**
 * @brief Configurator that discretises DEFAULT tasks into fixed-length segments. Basically, this is an implementation of classic A* search
 * 
 */
class DiscreteConfigurator : public virtual FocusedConfigurator{
public:
/**
 * @brief In discrete configurator, Di is previous state's Dn, or the goal, if null
 */
Disturbance getDisturbance(TransitionSystem&g, vertexDescriptor v, b2World & world, const Direction & dir, const b2Transform& start)override;

Robot makeRobot(b2World & w, const Task & task)override;

/**
 * @brief Sets a time limit to DEFAULT tasks corresponding to the amount of time estimated
 * to complete a forward move of length simulationStep
 */
float remainingSimulationTime(const Task *const t=NULL)override;

/**
 * @brief In discrete configurator, matches must be exact and
 * no signal to replan if Task is successful
 * 
 * @param s state to match
 * @param dir direction of the task
 * @param match_type type of desired match
 * @param _sd pointer to state difference
 * @return VertexMatch 
 */
VertexMatch findMatch(State s, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, StateDifference * _sd=NULL, vertexDescriptor src=TransitionSystem::null_vertex())override;

StateMatcher::MATCH_TYPE desiredMatch() override {
	return StateMatcher::MATCH_TYPE::_TRUE;
}
/**
 * @brief In discrete configurator, successful DEFAULT tasks can transition to DEFAULT, LEFT, RIGHT
 * 
 * @param v vertex of source state
 * @param d direction of the Task
 * @param src source vertex of v
 */
void transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src) override;

/**
 * @brief Just adds to closed set
 */
bool closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v)override;


void backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src, vertexDescriptor startRecycle)override;

std::vector<Direction> partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve)override;

bool shouldPartiallyExplore(const std::vector<edgeDescriptor>& oe, std::pair<bool, edgeDescriptor> ve)override;

bool canPropagate(vertexDescriptor v) override;

bool canReassignOutcome(vertexDescriptor v) override;

bool propagateD(vertexDescriptor v1, vertexDescriptor v0)override;

float customSimulationStep(vertexDescriptor v=TransitionSystem::null_vertex())override{
	return simulationStep;
}

};

// /**
//  * @brief Plans by simply looking ahead at the possible Tasks and choosing the best sequence.
//  * Does not process states, e.g. with split
//  */
// class SimplestConfigurator: public FocusedConfigurator{
// 	int simulationStep=BOX2DRANGE;

// 	Robot makeRobot(b2World & world, const b2Transform & start){
// 		return Configurator::makeRobot(world, start);
// 	}

// 	Disturbance getDisturbance(TransitionSystem&g, vertexDescriptor v, b2World & world, const Direction & dir, const b2Transform& start){
//     	return g[v].Dn;
// 	}

// 	void backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src=MOVING_VERTEX, vertexDescriptor startRecycle=MOVING_VERTEX);



// };
#endif