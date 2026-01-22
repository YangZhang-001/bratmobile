#include "graphTools.h"

b2Transform State::start_from_Di()const{
	if (Di.getAffIndex()==NONE){
		return b2Transform_inf;
	}
	return b2MulT(start, Di.pose());
}

b2Transform State::start_from_Dn()const{
	if (Dn.getAffIndex()==NONE){
		return b2Transform_inf;
	}
	return b2MulT(start, Dn.pose());
}

b2Transform State::end_from_Dn()const{
	if (Dn.getAffIndex()==NONE){
		return b2Transform_inf;
	}
	return b2MulT(endPose, Dn.pose());

}

b2Transform State::end_from_Di()const{
	if (Di.getAffIndex()==NONE){
		return b2Transform_inf;
	}
	return b2MulT(endPose, Di.pose());
}

float State::distance()const{
	return (b2help::InvMul(endPose, start)).p.Length();
}

b2Transform State::travel_transform(){
	//return start-endPose;
	return b2help::InvMul(endPose, start);
}

bool Edge::enableOverride(){
	if (step==0){
		overrideZeroSteps=true;
		return true;
	}
	return false;
}





float StateDifference::get_sum(int mt){
	if (mt==StateMatcher::_FALSE || mt==StateMatcher::ANY){
		return 10000;
	}
	else if (mt==StateMatcher::_TRUE){
		return sum();
	}
	else if (mt==StateMatcher::D_NEW){
		return sum_D(Dn);
	}
	else if (mt==StateMatcher::POSE){
		return sum_r();
	}
	else if (mt==StateMatcher::D_INIT){
		return sum_D(Di);
	}
	else if (mt==StateMatcher::ABSTRACT){
		return sum_D(Dn) +sum_D(Di);
	}
}

void StateDifference::init(const State& s1, const State& s2){ //observed, desired
	pose.p.x= s1.endPose.p.x-s2.endPose.p.x; //endpose x
	pose.p.y=s1.endPose.p.y-s2.endPose.p.y; //endpose y
	pose.q.Set(angle_subtract(s1.endPose.q.GetAngle(), s2.endPose.q.GetAngle()));
	if (s1.Dn.getAffIndex()!= s2.Dn.getAffIndex()){
		fill_invalid_bodyfeatures(Dn);
	}
	else{
		fill_valid_bodyfeatures(Dn, s1, s2, DN);
	}
	if (s1.Di.getAffIndex()!=s2.Di.getAffIndex()){
		fill_invalid_bodyfeatures(Di);
	}
	else{
		fill_valid_bodyfeatures(Di, s1, s2, DI);
	}

}

void StateDifference::fill_invalid_bodyfeatures(BodyFeatures & bf){
	bf.pose.p.x=10000;
	bf.pose.p.y=10000;
	bf.pose.q.Set(MAX_ANGLE_ERROR);
	bf.halfLength=10000;
	bf.halfWidth=10000;
}

void StateDifference::fill_valid_bodyfeatures(BodyFeatures & bf, const State& s1, const State& s2, WHAT_D_FLAG flag){
	b2Transform p1=b2Transform_zero, p2=b2Transform_zero;
	if (flag==DI){
		if (s1.Di.getAffIndex()==NONE && s2.Di.getAffIndex()==NONE){
			return;
		}
		p1=s1.start_from_Di();
		p2=s2.start_from_Di();
		bf.halfWidth=(s1.Di.bodyFeatures().halfWidth-s2.Di.bodyFeatures().halfWidth);
		bf.halfLength=(s1.Di.bodyFeatures().halfLength-s2.Di.bodyFeatures().halfLength);
	}
	else if (flag==DN){
		if (s1.Dn.getAffIndex()==NONE && s2.Dn.getAffIndex()==NONE){
			return;
		}
		p1=s1.end_from_Dn();
		p2=s2.end_from_Dn();
		bf.halfWidth=(s1.Dn.bodyFeatures().halfWidth-s2.Dn.bodyFeatures().halfWidth);
		bf.halfLength=(s1.Dn.bodyFeatures().halfLength-s2.Dn.bodyFeatures().halfLength);

	}
	bf.pose.p.x= p1.p.x - p2.p.x; //disturbance x
	bf.pose.p.y= p1.p.y - p2.p.y; //disturbance y
	float pose_q= angle_subtract(p1.q.GetAngle(), p2.q.GetAngle());
	bf.pose.q.Set(pose_q);

}


void gt::fill(simResult sr, State* s, Edge* e){
	if (NULL!=s){
		s->Dn = sr.collision;
		s->endPose = sr.endPose;
		s->outcome = sr.resultCode;
		s->filled=true;
	}
	if (NULL!=e){
		e->step = gt::simToMotorStep(sr.step);

	}
}

int gt::simToMotorStep(int simStep){
	float result =std::floor(float(simStep)/(HZ*MOTOR_CALLBACK)+0.5);
	return int(result);
}

int gt::distanceToSimStep(const float& s, const float& ds){
	float time=s/ds; //ds:1=s:time
	int sim_step=(time*HZ);
	return sim_step;
}


void gt::update(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, int it){
	if (e==edgeDescriptor()){
		return;
	}
	if (!current){
		g[e].step = sk.second.step;
	}
	g[e.m_target].Dn = sk.first.Dn;
	if(sk.first.label==g[e.m_target].label){
		g[e.m_target].endPose = sk.first.endPose;
	}
	g[e.m_target].options = sk.first.options;
	g[e.m_target].nObs++;
	if (e.m_source!=e.m_target){
		g[e.m_target].start=sk.first.start;
	}
	if (!g[e.m_target].visited()){
		g[e.m_target].phi=sk.first.phi;
	}
	g[e.m_target].filled=sk.first.filled;
	g[e].it_observed=it;
}

void gt::set(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, int it){
	if (g[e.m_target].filled){
		return;
	}
	update(e, sk, g, current, it);
	g[e.m_target].outcome = sk.first.outcome;
}

std::vector <edgeDescriptor> gt::outEdges(TransitionSystem&g, vertexDescriptor v, Direction d){
	std::vector <edgeDescriptor> result;
	auto es = boost::out_edges(v, g);
	for (auto ei = es.first; ei!=es.second; ++ei){
		if (g[(*ei).m_target].direction == d || d==UNDEFINED){
			result.push_back(*ei);
		}
	}
	return result;
}


std::pair< bool, edgeDescriptor> gt::getMostLikely(TransitionSystem& g, std::vector <edgeDescriptor> oe, int it){
	std::pair< bool, edgeDescriptor> mostLikely(false, edgeDescriptor());
	float prob=-1;
	for (edgeDescriptor e:oe){
		if (g[e].weighted_probability(it)>prob){
			mostLikely.second=e;
			prob=g[e].weighted_probability(it);
		}
	}
	mostLikely.first=!oe.empty();
	return mostLikely;
}


Disturbance gt::getExpectedDisturbance(TransitionSystem& g, vertexDescriptor v, Direction d, int it){
	std::vector<edgeDescriptor> oe=outEdges(g, v, d);
	Disturbance result=Disturbance();
	if (oe.empty()){
		return result;
	}
	std::pair<bool,edgeDescriptor> mostLikely=getMostLikely(g, oe, it);
	if (mostLikely.first){
		result=g[mostLikely.second.m_target].Dn;
	}
	return result;

}
std::pair <bool,edgeDescriptor>  gt::visitedEdge(const std::vector <edgeDescriptor> &es, TransitionSystem& g, vertexDescriptor cv){
	std::pair <bool,edgeDescriptor> result(false, edgeDescriptor());
	std::vector<edgeDescriptor> possible_solutions;
	for (edgeDescriptor e:es){
		if ((g[e.m_source].visited() & g[e.m_target].visited()) || e.m_target==DUMMY){ 
			// result.second=e;
			// result.first=true;
			possible_solutions.push_back(result.second);
			//break;
			//return result;
		}
	}
	if (!possible_solutions.empty()){
	//pick most recent...
		struct CompareIteration{
			TransitionSystem & g;
			public:
			CompareIteration(TransitionSystem & _g):g(_g){}
			bool operator()(edgeDescriptor e1, edgeDescriptor e2){
				return g[e1].it_observed<=g[e2].it_observed;
			}
		};
		auto it_recent=std::max_element(possible_solutions.begin(), possible_solutions.end(), CompareIteration(g));
		if (it_recent!=possible_solutions.end()){
			result.second=*it_recent;
			result.first=true;
		}
	}
	return result;
}



std::pair <edgeDescriptor, bool> gt::add_edge(const vertexDescriptor & u, const  vertexDescriptor & v, TransitionSystem& g, const int &it, Direction d){
	std::pair <edgeDescriptor, bool> result=boost::edge(u, v, g);
	if (u==v){
		if (d==UNDEFINED){
			return result;
		}
	}
	auto oe=outEdges(g, u, d);
	for (auto e:oe){
		if (g[v].Dn == g[e.m_target].Dn&& u!=v ){

			return result;
		}
	}
	result=boost::add_edge(u, v, g);
	if (d==DEFAULT && result.second){
		float delta=0;
		auto values =(*default_kinematics.find(d)).second;
		g[result.first].step=distanceToSimStep(g[v].distance(), values.first);
		//printf("u=%i, v=%i, step=%i\n", u, v, g[result.first].step);
	}
	g[result.first].it_observed=it;
	return result;
}

bool gt::check_edge_direction(const std::pair<edgeDescriptor, bool> & ep, TransitionSystem& g, Direction d){
	bool result=false;
	if (ep.second){
		result=g[ep.first.m_target].direction==d;
	}
	return result;
}



std::vector<vertexDescriptor>::iterator gt::to_task_end(edgeDescriptor& e, TransitionSystem &g, const std::vector<vertexDescriptor> & plan,  std::vector<vertexDescriptor>::iterator it){
edgeDescriptor e_start=e;
std::pair<edgeDescriptor, bool> ep;
do{
	ep=boost::edge(*it, *(it+1), g);
	if (!ep.second){
		break;
	}
	else{
		e=ep.first;
	}
	if (g[e.m_target].direction==g[e_start.m_target].direction){
		it++;
	}
}while(g[e.m_target].direction==g[e_start.m_target].direction &&
		 it != plan.end() && it!=(plan.end()-1)               &&
		 (g[e.m_target].Di==g[e_start.m_source].Di)
		 );

return (it);
}




bool StateMatcher::match_equal(const MATCH_TYPE& candidate, const MATCH_TYPE& desired){
	bool result=false;
	switch (desired){ //the desired match
		case ANY:
			if (int(candidate)!=int(_FALSE)){
				result=true;
			}
			break;
		case POSE:
			if (int(candidate)==int(_TRUE) || int(candidate) == int(POSE)){
				result=true;
			}
			break;
		case ABSTRACT:
			if (int(candidate)==_TRUE || int(candidate) ==ABSTRACT){
				result=true;
			}
			break;
		case _FALSE:
			result=int(candidate)==int(desired);
			break;
		case _TRUE:
			result=int(candidate)==int(desired);
			break;
		case DN_SHAPE:
			result =(int(candidate)==int(desired))|| int(candidate)==_TRUE || int(candidate)==ABSTRACT || candidate== D_NEW;
			break;			
		case DN_POSE:
			result =(int(candidate)==int(desired))|| candidate==_TRUE || int(candidate)==ABSTRACT || candidate== D_NEW;
			break;			
		case DI_SHAPE:
			result =(int(candidate)==int(desired))|| candidate==_TRUE || candidate==ABSTRACT || candidate== D_INIT;
			break;			
		case DI_POSE:
			result =(int(candidate)==int(desired))|| candidate==_TRUE || candidate==ABSTRACT || candidate== D_INIT;
			break;			
		default:
			result =(int(candidate)==int(desired)) ||candidate==_TRUE || candidate==ABSTRACT ;
			break;
	}
	return result;
}



StateMatcher::MATCH_TYPE StateMatcher::isMatch(const StateDifference& sd, const  Threshold &threshold, float endDistance){
	float coefficient=get_coefficient(endDistance);
	StateMatcher::StateMatch match(sd, threshold, coefficient);
    return match.what();
}

StateMatcher::MATCH_TYPE StateMatcher::isMatch(const State & s, const State &candidate, const Threshold& threshold, const State *src, StateDifference*_sd){
	//src is the source of candidate
	StateDifference sd(s, candidate);
	// float stray=0;
	// if ((stray>error.endPosition && s.label==candidate.label)){ //
	// 	sd.pose.p.x=10000; // now pose will not be matched
	// 	sd.pose.p.y=10000;
	// 	sd.pose.q.Set(MAX_ANGLE_ERROR);
	// }
	if (NULL!=_sd){
		*_sd=sd;
	}
    return isMatch(sd, threshold, s.endPose.p.Length()) ;
}


// std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> StateMatcher::match_vertex(TransitionSystem g, vertexDescriptor src, Direction d, State s, StateMatcher::MATCH_TYPE mt){
//     std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::MATCH_TYPE::_FALSE, TransitionSystem::null_vertex());
// 	auto edges= boost::out_edges(src, g);
// 	for (auto ei=edges.first; ei!=edges.second; ++ei){
// 		//MATCH_TYPE match = isMatch(s, g[ei.dereference().m_source]);
// 		MATCH_TYPE match = isMatch(s, g[ei.dereference().m_target]);
// 		if (g[(*ei).m_target].direction && match_equal(match, mt)){
// 			result.first=match;
// 			result.second=(*ei).m_target;
// 			break;
// 		}
// 	}
//     return result;
// }

float StateMatcher::get_coefficient(const float & endDistance){
	float coefficient=1.0;
	// if (endDistance>COEFFICIENT_INCREASE_THRESHOLD){
	// 	float scale=1+(endDistance-COEFFICIENT_INCREASE_THRESHOLD);// /.9
	// 	//coefficient+=(endDistance-COEFFICIENT_INCREASE_THRESHOLD)/2;
	// 	coefficient*=scale; //it's a bit high but need for debugging (before *1.2)
	// }
	return coefficient;
}




