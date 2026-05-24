#ifndef GENERAL_H
#define GENERAL_H

#include <opencv2/calib3d.hpp> //LMEDS
#include <vector>
#include <utility>                   // for std::pair
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp> //property map
#include <boost/graph/copy.hpp>
#include <utility>
#include "disturbance.h"
#include "box2d_helpers.h"

const float NAIVE_PHI=10.0;

class Task;
enum VERTEX_LABEL {UNLABELED, MOVING, ESCAPE, ESCAPE2};

/**
 * @brief Compares the last float in a tuple
 * 
*/
struct CompareValue{
	CompareValue()=default;
	template <class V, class M>
	bool operator()(const std::tuple<V,M, float> & p1, const std::tuple<V, M, float> &p2) const{
		return std::get<2>(p1)< std::get<2>(p2);
	}
};

/**
 * @brief Edges connecting states in the transition system
 * 
 */
struct Edge{
	float probability=1.0;
	int step=0;
	int it_observed=-1; //last iteration where this edge was observed
	bool overrideZeroSteps=false; //set to true if an edge with no steps should not be considered a self-edge

	Edge()=default;

	float weighted_probability(int it){
		float result=0;
		if (it_observed>=0){
			result=probability*float(it_observed)/float(it);
		}
		return result;
	}

	/**
	 * @brief If this edge has zero steps, it sets override to true
	 * 
	 * @return true if override was changed to true
	 * @return false if no changes were made
	 */
	bool enableOverride();
};

/**
 * @brief Hybrid states in the transition system
 * 
 */
struct State{
	Disturbance Di; //initial Disturbance
	Disturbance Dn; //new Disturbance
	b2Transform endPose = b2Transform_zero, start = b2Transform_zero; 
	simResult::resultType outcome=simResult::successful;
	std::vector <Direction> options;
	bool filled =0;
	int nObs=0;
	float phi=NAIVE_PHI; //arbitrarily large phi
	VERTEX_LABEL label=VERTEX_LABEL::UNLABELED;
	Direction direction=DEFAULT;


	State()=default;

	State(const b2Transform &_start): start(_start){}

	State(const b2Transform &_start, const Disturbance& di, const Direction & dir): start(_start), Di(di), direction(dir){}

	/**
	 * @brief Whether this state has been visited in exploration using evaluation function phi as proxy
	 */
	bool visited(){
		return phi<NAIVE_PHI;
	}

	/**
	 * @brief Sets the phi value to its naive value (call to visited() will return false)
	 * 
	 */
	void resetVisited(){
		phi=NAIVE_PHI;
	}

	/**
	 * @brief Return transformation from start to Di position
	 */
	b2Transform start_from_Di()const;

	/**
	 * @brief Return transformation from start to Dn position
	 */
	b2Transform start_from_Dn()const;

	/**
	 * @brief Return transformation from end to Di position
	 */
	b2Transform end_from_Dn()const;

	/**
	 * @brief Return transformation from end to Di position
	 */
	b2Transform end_from_Di()const;

	float distance()const;

	b2Transform travel_transform();

	bool isTurning()const{
		return direction==LEFT || direction==RIGHT;
	}

	bool isGoingStraight()const{
		return direction==DEFAULT || direction==STOP;
	}



	// float gamma(){
	// 	Angle a(atan2(end_from_Dn().p.y, end_from_Dn().p.y));
	// 	Distance d(end_from_Dn().p.Length());
	// 	return getStanda
	// }

};


/**
 * @brief Contains the differences between the contiuous components of two hybrid states
 * 
 */
struct StateDifference{
	b2Transform pose=b2Transform_zero;
	BodyFeatures Di, Dn;
	enum WHAT_D_FLAG{DI, DN};

	StateDifference()=default;

	StateDifference(const State& s1, const State& s2){
		init(s1, s2);
	}

	float sum(){
		return sum_r()+sum_D(Di)+ sum_D(Dn);
	}

	/**
	 * @brief Total difference in "global" robot pose 
	 */
	float sum_r(){
		return fabs(pose.p.x)+fabs(pose.p.y)+fabs(pose.q.GetAngle());
	}

	/**
	 * @brief Total difference in disturbance pose
	 * 
	 * @param bf body features of disturbance
	 */
	float sum_D_pos(const BodyFeatures& bf){
		return fabs(bf.pose.p.x)+fabs(bf.pose.p.y)+bf.pose.q.GetAngle();
	}

	/**
	 * @brief Total difference in disturbance shape
	 * 
	 * @param bf body features of disturbance
	 */
	float sum_D_shape(const BodyFeatures& bf){
		return fabs(bf.width())+fabs(bf.length());
	}

	/**
	 * @brief Total difference in disturbance pose and shape combined
	 * 
	 * @param bf body features of disturbance
	 */
	float sum_D(const BodyFeatures& bf){
		return sum_D_pos(bf)+sum_D_shape(bf);
	}
	
	/**
	 * @brief Returns difference in certain aspects we want to match between states
	 * 
	 * @param mt the match type
	 */
	float get_sum(int mt);

	/**
	 * @brief Initialises the difference object using two states we want to compare
	 * 
	 * @param s1 
	 * @param s2 
	 */
	void init(const State& s1,const State& s2);

	/**
	 * @brief Fills bf match with large values indicating no match
	 * 
	 */
	void fill_invalid_bodyfeatures(BodyFeatures &);

/**
 * @brief Fills the bodyfeatures match with differences between disturbances in the two states. Differences
 * are calculated in the position local to the robot in that state, not globally
 * 
 * @param bf body features to fill
 * @param s1 
 * @param s2 
 * @param flag whether it's a Di or Dn
 */
	void fill_valid_bodyfeatures(BodyFeatures & bf, const State& s1, const State& s2, WHAT_D_FLAG flag);
};


typedef boost::adjacency_list<boost::setS, boost::vecS, boost::bidirectionalS, State, Edge> TransitionSystem;

typedef boost::graph_traits<TransitionSystem>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<TransitionSystem>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_iterator edgeIterator;

//SPECIAL VERTICES
/**
 * @brief vertex reprensenting instantaneous position of the robot relative to itself
 * Trivial: in the graph it's always located at the origin with an orientation of 0 degrees, and
 * should always be connected to the vertex representing the current state.
 * 
 */
const vertexDescriptor MOVING_VERTEX=0; 

/**
* @brief first vertex added to transition system, by default represents the instantaneous state 
* of the robot when it first starts planning, used so that any branches expanded out of it share the same root
* once the moving vertex attaches to another state
*/
const vertexDescriptor DUMMY=1;


/**
 * @brief Used as a predicate, gives info on whether a vertex is the current vertex
 * 
 */
struct is_not_v{
	is_not_v(){}
	/**
	 * @brief Constructor assigns current vertex
	 * 
	 * @param _cv the current vertex
	 */
	is_not_v(vertexDescriptor _cv): cv(_cv){}

	bool operator()(edgeDescriptor e){
		return e.m_target!=cv;
	}	

	private:
	vertexDescriptor cv=0;
};

/**
 * @brief Predicate: gives info on whether a vertex has connections or is a singleton
 * 
 */
struct Connected{
	Connected(){}
	Connected(TransitionSystem * ts): g(ts){}
	
	bool operator()(const vertexDescriptor& v)const{
	 	bool in= boost::in_degree(v, *g)>0;
		bool out =boost::out_degree(v, *g)>0;
	 	return (in || out) || v==0 ;
	}
private:
TransitionSystem * g=NULL;
};



namespace gt{

	/**
	 * @brief Fills a state-edge usign the box2d simulation result
	 * 
	 * @param sr simulation result
	 * @param s state pointer
	 * @param e edge pointer
	 */
	void fill(simResult sr, State* s=NULL, Edge* e=NULL);

	/**
	 * @brief returns the number of motor steps corresponding to @param simStep simulation steps (uses macros in const.h)
	 */
	int simToMotorStep(int simstep);

	/**
	 * @brief Returns simulation steps necessary to cover a certain distance given a velocity
	 * 
	 * @param s distance
	 * @param ds velocity
	 * @return int 
	 */
	int distanceToSimStep(const float& s, const float& ds);

	/**
	 * @brief Updates the target of e=(src, target)
	 * 
	 * @param e the edge whose target will be updated
	 * @param sk a state-edge observation
	 * @param g the transitionsystem
	 * @param current is this edge the current edge? (if yes the step is not updated)
	 * @param it iteration
	 */
	void update(edgeDescriptor e,  std::pair <State, Edge> sk, TransitionSystem& g, bool current, int it); 
	/**
	 * @brief Resets the target of e=(src, target). The difference with update is that it resets the outcome and it doesn't update if the vertex is already filled
	 * 
	 * @param e the edge whose target will be updated
	 * @param sk a state-edge observation
	 * @param g the transitionsystem
	 * @param current is this edge the current edge? (if yes the step is not updated)
	 * @param it iteration
	 */
	void set(edgeDescriptor e,  std::pair <State, Edge>sk, TransitionSystem&, bool current, int it);


	std::pair< bool, edgeDescriptor> getMostLikely(TransitionSystem&,std::vector<edgeDescriptor>, int);

	std::vector <edgeDescriptor> outEdges(TransitionSystem&, vertexDescriptor, Direction); //returns a vector containing all the out-edges of a vertex which have the specified direction

	Disturbance getExpectedDisturbance(TransitionSystem&, vertexDescriptor, Direction, int);

	/**
	 * @brief Returns a valid visited edge, if present
	 * 
	 * @param es vector of out-edges
	 * @param g transitionSystem
	 * @param cv current vertex 
	 * @return std::pair <bool,edgeDescriptor> (is edge valid, visited edge). Returns true if cv is the source of any of the outedges
	 */
	std::pair <bool,edgeDescriptor> visitedEdge(const std::vector <edgeDescriptor>& es, TransitionSystem& g, vertexDescriptor cv=TransitionSystem::null_vertex());


	std::pair <edgeDescriptor, bool> add_edge(const vertexDescriptor&, const vertexDescriptor &, TransitionSystem&, const int &, Direction d=UNDEFINED); //wrapper around boost function, disallows edges to self

	/**
	 * @brief Checks that 
	 * 
	 * @return true 
	 * @return false 
	 */
	bool check_edge_direction(const std::pair<edgeDescriptor, bool> &, TransitionSystem&, Direction);
	/**
	 * @brief Travels in the graph to find the end of the task in e.m_target. In case the task
	 * is only one vertex, e will be changed to the next edge not belonging to the task, and it is an iterator
	 * in the plan vector to the next vertex in the plan 
	 * @param e edge whose target is the task we want to find the end of
	 * @param g the graph
	 * @param plan the plan
	 * @param it iterator to the vertex e.m_target (in plan vector)
	 * @return std::vector<vertexDescriptor>::iterator 
	 */
	std::vector<vertexDescriptor>::iterator to_task_end(edgeDescriptor &e, TransitionSystem &g, const std::vector<vertexDescriptor> & plan, std::vector<vertexDescriptor>::iterator it); //in a vector, finds vertices belonging to the same task and skips to the end fo the task



}

struct InPlan{
	InPlan()=default;

	InPlan(std::vector <vertexDescriptor>* _p):plan(_p){}

	bool operator()(const edgeDescriptor & e)const{
		return (check_vector_for(*plan, e.m_target)!=plan->end());
	}

	private:
	std::vector<vertexDescriptor> *plan;
};



struct NotSelfEdge{
	NotSelfEdge()=default;
	NotSelfEdge(TransitionSystem * _g): g(_g){}

	bool operator()(const edgeDescriptor & e) const {
		bool not_self= e.m_source!=e.m_target && (*g)[e].step!=0  ; 
		// if (e.m_source==e.m_target){
		// 	auto def_kin =(*default_kinematics.find((*g)[e.m_target].direction)).second;

		// }
		return not_self;
	}
	private:
	TransitionSystem * g=NULL;
};

/**
 * @brief Predicate: gives info on whether an edge is worth keeping. Namely,
 * the edge is either not a self-edge (the vertex is not connected to itself)
 * and if it is, the edge is not trivial (i.e. simulating this self-transition takes more than 0 simulation steps)
 * 
 */
struct ViableEdge{
	ViableEdge()=default;
	ViableEdge(TransitionSystem * _g): g(_g){}

	bool operator()(const edgeDescriptor & e) const {
		NotSelfEdge nse(g);
		bool not_self= nse(e) || e.m_source==e.m_target && (*g)[e].step!=0 && (*g)[e].it_observed>=0;
		return not_self;
	}
	private:
	TransitionSystem * g=NULL;
};

struct InviableEdge{
	InviableEdge()=default;
	InviableEdge(TransitionSystem * _g): g(_g){}

	bool operator()(const edgeDescriptor & e) const {
		ViableEdge ve(g);
		return !ve(e);
	}

private:
TransitionSystem * g=NULL;
};

struct SameIteration{
	SameIteration()=delete;
	SameIteration(TransitionSystem & _g, int _i): g(_g), iteration(_i){}

	bool operator()(const edgeDescriptor & e) const {
		return g[e].it_observed==iteration;
	}
	private:
	TransitionSystem & g;
	int iteration=-1;
};



typedef boost::filtered_graph<TransitionSystem, ViableEdge, Connected> FilteredTS;


class StateMatcher{
	public:
		//@brief {_FALSE=0, D_NEW=2, DN_POSE=3, _TRUE=1, ANY=4, D_INIT=5, ABSTRACT=6, DI_POSE=7, DN_SHAPE=8, DI_SHAPE=9, POSE=10};
		enum MATCH_TYPE {_FALSE, D_NEW, DN_POSE, _TRUE, ANY, D_INIT, ABSTRACT, DI_POSE, DN_SHAPE, DI_SHAPE, POSE, D_SHAPES};

		float mu=0.001;
	    StateMatcher()=default;

		struct StateMatch{

			bool exact(){
				return pose() && abstract();
			}

			bool pose(){
				return position && angle;
			}

			bool abstract(){  //states match Di and Dn regardless of objective pose
				return Di_exact() && Dn_exact();
			}

			bool Di_exact(){
				return Di_position && Di_angle && Di_shape;
			}

			bool Dn_exact(){
				return Dn_position && Dn_angle &&  Dn_shape;
			}

			bool Di_pose(){
				return Di_position &&  Di_angle;
			}

			bool Dn_pose(){
				return Dn_position && Dn_angle;
			}

			bool shape_Di(){
				return  Di_shape;
			}

			bool shape_Dn(){
				return Dn_shape;
			}
			bool Dn(){
				return Dn_shape && Dn_pose();
			}

			bool Di(){
				return Di_shape && Di_pose();
			}

			bool D_shapes(){
				return Dn_shape && Di_shape;
			}

			StateMatch() =default;

			StateMatch(const StateDifference& sd,const Threshold& threshold, float coefficient=1){
				position = sd.pose.p.Length()<(threshold.for_robot_position()*coefficient);
				angle=fabs(sd.pose.q.GetAngle())<threshold.for_robot_angle();
				Bundle Dn=threshold.for_Dn(), Di=threshold.for_Di();
				Dn_position= sd.Dn.pose.p.Length()<(Dn.radius()*coefficient);
				Di_position= sd.Di.pose.p.Length()<(Di.radius()*coefficient);
				Dn_angle=fabs(sd.Dn.pose.q.GetAngle())<Dn.get_angle();
				Di_angle=fabs(sd.Di.pose.q.GetAngle())<Di.get_angle();
				bool Dn_below_threshold_w=fabs(sd.Dn.width())<(Dn.get_width()*coefficient*2);
				bool Dn_below_threshold_l=fabs(sd.Dn.length())<(Dn.get_length()*coefficient*2);
				bool Di_below_threshold_w=fabs(sd.Di.width())<(Dn.get_width()*coefficient*2);
				bool Di_below_threshold_l=fabs(sd.Di.length())<(Dn.get_length()*coefficient*2);
				Dn_shape= Dn_below_threshold_l && Dn_below_threshold_w;
				Di_shape= Di_below_threshold_l && Di_below_threshold_w;

			}

			StateMatcher::MATCH_TYPE what(){
				if (exact()){ //match position and disturbance
					return _TRUE;
				}
				else if (abstract()){
					return ABSTRACT;
				}
				else if (Dn_exact()){
					return D_NEW;
				}
				else if (Di_exact()){
					return D_INIT;
				}	
				else if (D_shapes()){
					return D_SHAPES;
				}			
				else if (Dn_pose()){
					return DN_POSE;
				}
				else if (Di_pose()){
					return DI_POSE;
				}				
				else if (shape_Dn()){
					return DN_SHAPE;
				}
				else if (shape_Di()){
					return DI_SHAPE;
				}
				else{
					return _FALSE;
				}
			}

			private:

			bool position=false;
			bool angle=false;
			bool Di_position=false;
			bool Di_angle=false;
			bool Di_shape=false;
			bool Dn_position=false;
			bool Dn_angle=false;
			bool Dn_shape=false;
		};

		bool match_equal(const MATCH_TYPE& candidate, const MATCH_TYPE& desired);

		MATCH_TYPE isMatch(const StateDifference &, const Threshold &, float endDistance=0); //endDistance=endpose

		MATCH_TYPE isMatch(const State & s, const State& candidate, const  Threshold &, const State* src=NULL, StateDifference * _sd=NULL); //first state: state to find a match for, second state: candidate match

		//std::pair<MATCH_TYPE, vertexDescriptor> match_vertex(TransitionSystem, vertexDescriptor, Direction, State, StateMatcher::MATCH_TYPE mt=StateMatcher::_TRUE); //find match amoung vertex out edges

		float get_coefficient(const float &);
	private:


	const float COEFFICIENT_INCREASE_THRESHOLD=0.0;
};

typedef std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> VertexMatch;

#endif