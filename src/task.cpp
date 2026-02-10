#include "task.h"


b2Fixture * GetSensor( b2Body * body){
	for (b2Fixture * f=body->GetFixtureList(); f;f=f->GetNext()){
		if (f->IsSensor()){
			return f;
		}
	}
	return NULL;
}

b2Body * GetDisturbance(b2World * w){
	for (b2Body * b=w->GetBodyList();b;b=b->GetNext()){
		if (b->GetUserData().pointer==DISTURBANCE_FLAG){
			return b;
		}
	}
	return NULL;
}


bool overlaps(b2Body * robot, const Disturbance *const disturbance){
	b2Fixture * sensor=GetSensor(robot);
	if (sensor==NULL){
		return true;
	}
	if (disturbance==NULL || disturbance->getAffIndex()!= AVOID ){
		return true;
	}
	b2Transform robot_pose=robot->GetTransform(), d_pose= disturbance->pose();
	b2PolygonShape d_shape;
	d_shape.SetAsBox(disturbance->bf.halfWidth, disturbance->bf.halfLength, b2Vec2(0,0), 0);
	return b2TestOverlap(sensor->GetShape(), 0, &d_shape, 0,robot_pose, d_pose);
}

bool overlaps(const b2PolygonShape& box, const Disturbance * const d, const b2Transform& robot_pose){
	bool result=true;
	if (!box.m_radius || NULL==d ){
		return result;
	}
	if (d->getAffIndex()!=AVOID){
		return result;
	}
	b2PolygonShape d_shape;
	d_shape.SetAsBox(d->bf.halfWidth, d->bf.halfLength, b2Vec2(0,0), 0);
	return b2TestOverlap(&box, 0, &d_shape, 0,robot_pose, d->bf.pose);

}

void world_cleanup(b2World & _world){
	for (b2Body * b = _world.GetBodyList(); b!=NULL;b = b->GetNext()){ // 
		_world.DestroyBody(b);
	}
}


simResult Task::bumping_that(b2World & _world, int iteration, b2Body * robot, float remaining){ //CLOSED LOOP CONTROL, og return simreult
	simResult result=simResult(simResult::resultType::successful);
	result.endPose = start;
	Listener listener(&disturbance);
	int _count=_world.GetBodyCount(), stepb2d=0;;
	_world.SetContactListener(&listener);	
	FILE * robotPath;
	if (DEBUG){
		sprintf(planFile, "/tmp/robot%04i.txt", iteration);
		robotPath = fopen(planFile, "a");
	}
	float theta = start.q.GetAngle();
	b2Vec2 instVelocity = {0,0};		
	for (stepb2d; stepb2d < (HZ*remaining); stepb2d++) {
		instVelocity.x = action.getLinearSpeed()*cos(theta);
		instVelocity.y = action.getLinearSpeed()*sin(theta);
		robot->SetLinearVelocity(instVelocity);
		robot->SetAngularVelocity(action.getOmega());
		robot->SetTransform(robot->GetPosition(), theta);
		if (DEBUG){
			fprintf(robotPath, "%f\t%f\n", robot->GetPosition().x, robot->GetPosition().y); //save predictions/
		}
		bool out_x= fabs(robot->GetTransform().p.x)>=(BOX2DRANGE-0.001);
		bool out_y= fabs(robot->GetTransform().p.y)>=(BOX2DRANGE-0.001);
		bool out=(out_x || out_y ), overlap=overlaps(robot, &disturbance);
		if (!overlap){
			disturbance.invalidate();
		}
		if (bool ended=checkEnded(robot->GetTransform(), direction, false, robot).ended; ended || out){ //out
			bool keep_going_out_x=(fabs(robot->GetTransform().p.x+instVelocity.x) >fabs(robot->GetTransform().p.x))&&out_x;
			bool keep_going_out_y=(fabs(robot->GetTransform().p.y+instVelocity.y) >fabs(robot->GetTransform().p.y))&&out_y;
			if (ended){
				break;
			}
			if (keep_going_out_x || keep_going_out_y){
				break;
			}
		}
		_world.Step(1.0f/HZ, 3, 8); //time step 100 ms which also is alphabot callback time, possibly put it higher in the future if fast
		theta += action.getOmega()/HZ; //= omega *t
		if (listener.get_collisions().size()>0){ //
			int index = int(listener.get_collisions().size()/2);
			Disturbance collision = Disturbance(listener.get_collisions()[index]);
			result = simResult(simResult::resultType::crashed, collision);
			break;
		}
	}
	result.endPose = robot->GetTransform();
	result.step=stepb2d;
	world_cleanup(_world);
	if (DEBUG){
		fclose(robotPath);
	}
	return result;

}


// void Task::Correct::operator()(Action & action, int step){
// 	float tolerance = 0.1; //tolerance in radians/pi = just under 2 degrees degrees
// 	if (action.getOmega()!=0){ //only check every 2 sec, og || motorstep<1
// 		//printf("returning\n");
// 		return;
// 	}
// 	//printf("error buffer sum = %f, i=%f\n", p(), get_i());
// 	if (fabs(get_i())>tolerance){//& step>correction_rate
// 		float p_correction= ((p()/bufferSize)*kp)/2; //do not increase one wheel speed too much
// 		float i_correction= (get_i()*ki)/2; //do not increase one wheel speed too much
// 		float d_correction= (get_d()*kd)/2; //do not increase one wheel speed too much
// 			action.R -= p_correction+ i_correction; 
// 		 	action.L+= p_correction+ i_correction;
// 		if (action.L>1.0){
// 		action.L=1.0;
// 		}
// 		if (action.R>1.0){
// 			action.R=1;
// 		}
// 		if (action.L<(-1.0)){
// 			action.L=-1;
// 		}
// 		if (action.R<(-1.0)){
// 			action.R=-1;
// 		}
	
// 	}
// }

// float Task::Correct::errorCalc(Action a, double x){
// 	float result=0;
// 	if (a.getOmega()!=0){
// 		return result;
// 	}
// 	else{
// 		return sin(a.getOmega())*MOTOR_CALLBACK-float(x); //-ve error if robot goes R, +ve error if goes L
// 	}	

// }


// float Task::Correct::update(float e){
// 	float p0=p();
// 	p_buffer.erase(p_buffer.begin());
// 	p_buffer.push_back(e);
// 	float p1=p();
// 	mf.buffer.erase(mf.buffer.begin());
// 	mf.buffer.push_back(p1);
// 	d=p1-p0;
// 	i+=e;
// 	return p1;
// }


Direction Task::H(Disturbance ob, Direction d, bool topDown){
	if (ob.isValid()){
        if (ob.getAffIndex()==int(InnateAffordances::AVOID)){ //REACTIVE BEHAVIOUR
            if (d == Direction::DEFAULT & !topDown){ //REACTIVE BEHAVIOUR
                if (ob.getAngle(start)<0){//angle formed with robot at last safe pose
                    d= Direction::LEFT; //go left
                }
                else if (ob.getAngle(start)>0){ //angle formed with robot at last safe pose
                    d= Direction::RIGHT; //
                }   
                else{
                    int c = rand() % 2;
                    d = static_cast<Direction>(c);

                }
            }
        }
		else if (ob.getAffIndex()==int(InnateAffordances::PURSUE)){
			if (d == Direction::DEFAULT & !topDown){ //REACTIVE BEHAVIOUR
                if (ob.getAngle(start)<-.1){//angle formed with robot at last safe pose
                    d= Direction::RIGHT; //go left
                }
                else if (ob.getAngle(start)>0.1){ //angle formed with robot at last safe pose, around .1 rad tolerance
                    d= Direction::LEFT; //
                }   
            }
		}
}
    return d;
}



void Task::setEndCriteria(const Angle& angle, const Distance &distance){
	switch(disturbance.getAffIndex()){
		case PURSUE:{
			endCriteria.angle=Angle(0);
			endCriteria.distance = Distance(0+DISTANCE_ERROR_TOLERANCE);
		}
		break;
		default:
		endCriteria.distance = distance;

		endCriteria.angle = angle;
		if (direction==RIGHT){
			endCriteria.angle.set(-endCriteria.angle.get());
		}		
		break;
	}
	if (!action.getOmega()){
		endCriteria.angle.setValid(false);
	}
	else{
		endCriteria.distance.setValid(false);
	}
}

void Task::setEndCriteria(const Distance& distance){
	endCriteria.distance=distance;
}


EndedResult Task::checkEnded(b2Transform robotTransform, Direction dir,bool relax, b2Body * robot, std::pair<bool,b2Transform> use_start){ //self-ended , std::pair<bool,b2Transform> use_start
	if (dir==UNDEFINED){
		dir=direction;
	}
	b2Transform this_start= start;
	if (!use_start.first){
		this_start=use_start.second;
	}
	EndedResult r;
	Angle a;
	Distance d;
	b2Vec2 distance=this_start.p-robotTransform.p;
	if (round(distance.Length()*100)/100>=BOX2DRANGE){ //if length reached or turn
		r.ended =true;
	}
	if (disturbance.isValid()){
		//this shoudl be replaced by (d orientation == orientation+-PI/2), no robot position at alll
		b2Vec2 v = disturbance.getPosition() - robotTransform.p; //distance between disturbance and robot
		d= Distance(v.Length());
		if (action.getOmega()!=0){
			a =Angle(robotTransform.q.GetAngle());	
			if (isTurnFinished(robotTransform, dir)){
				if (disturbance.getAffIndex()==AVOID){
					disturbance.invalidate();
				}
				r.ended = 1;
			}
		}
		else if (getAffIndex()== int(InnateAffordances::NONE)){
			a =Angle(robotTransform.q.GetAngle());		
			r.ended = true;
		}
		else if (getAffIndex()==int(InnateAffordances::PURSUE)){
			a = Angle(disturbance.getAngle(robotTransform));
			//local level if D
			if (robot!=NULL){
				std::vector <b2Vec2> local_vertices=GetLocalPoints(disturbance.vertices(), robot);
				b2Vec2 pos_local=*(std::min_element(local_vertices.begin(), local_vertices.end(), CompareX()));
				r.ended=fabs(round(pos_local.x*100)/100)<=((endCriteria.distance.get()-0.001)/2); //-0.001 //was /2
			}
			else if (relax){
				Distance _d(RELAXED_DIST_ERROR_TOLERANCE);
				r.ended = d<=_d; 
			}
			else{
				r.ended = d.get()<=endCriteria.distance.get(); 
			}
		}
	}
	else{
		if (dir==LEFT || dir ==RIGHT){
		float angleL = this_start.q.GetAngle()+endCriteria.angle.get(), angleR = this_start.q.GetAngle()-endCriteria.angle.get();
		r.ended = (robotTransform.q.GetAngle()>=angleL || robotTransform.q.GetAngle()<=angleR);	
		}
		else if (dir==DEFAULT && getAffIndex()==AVOID){
			r.ended=true;
		}
	} 
	r.estimatedCost = endCriteria.getStandardError(a,d);
	return r;

}

EndedResult Task::checkEnded(const State& n,  Direction dir, bool relax, std::pair<bool,b2Transform> use_start){ //check error of node compared to the present Task
	EndedResult r;
	Angle a;
	Distance d;
	r = checkEnded(n.endPose, dir, relax,NULL, use_start);
	if (n.filled && n.outcome==simResult::crashed){
		r.estimatedCost+=2; //penalty for crashing
	}
	r.estimatedCost/=3;
	//r.estimatedCost+= endCriteria.getStandardError(a,d, n);
	return r;
}


bool Task::checkEnded(const b2PolygonShape &box , const b2Transform& robot_pose,Disturbance *dist_obs ){
	bool result=false;
	if (box.m_count<4){
		printf("no box!\n");
		return false;
	}
	if (dist_obs->getAffIndex()==NONE && direction==DEFAULT){
		if (start.p.Length()>=BOX2DRANGE){
			printf("far af\n");
			result=true;
		}
	}
	else if (dist_obs->getAffIndex()==PURSUE){ // && direction==DEFAULT
		b2Transform fromDi=from_Di(&b2Transform_zero);
		//Angle a(fromDi.q.GetAngle());
		Angle a(atan(fromDi.p.y/fromDi.p.x));
		Distance d(fromDi.p.x);
		result=endCriteria_met(a, d);
	}
	else if (dist_obs->getAffIndex()==AVOID ){ //|| action.getOmega()!=0
		if (box.m_radius==0 || action.getOmega()!=0){ //means that there is no goal 
			b2Transform fromDi_start=from_Di(&b2Transform_zero, dist_obs); //transform at start of task
			b2Transform fromDi_now=from_Di(&b2Transform_zero); 
			b2Transform inst_transform=b2MulT(fromDi_now, fromDi_start); //check how far Di has moved since start
			Angle a(-(fromDi_now.q.GetAngle()-(action.getTransform(LIDAR_SAMPLING_RATE/4).q.GetAngle()))); //avoid turning too much!
			float _distance=std::max(inst_transform.p.Length(), start.p.Length());
			Distance d(fabs(_distance));
			result=endCriteria_met(a, d);
		}
		else{
			result=!overlaps(box, &disturbance, robot_pose);
			FILE * box_file;
			box_file = fopen("/tmp/box_file.txt", "a");
			for (int i=0; i<box.m_count; i++){
				fprintf(box_file, "%f\t%f\n", box.m_vertices[i].x, box.m_vertices[i].y); //save predictions/		
			}
			fclose(box_file);
			if (result){
				printf("used box!");
			}
		}
	}
	return result;
}

b2Transform Task::from_Di(const  b2Transform* custom_start, Disturbance * d_obs){
	Disturbance *d;
	b2Transform _start=start;
	if (d_obs==NULL){
		d=&disturbance;
	}
	else{
		d=d_obs;
	}
    if (d->getAffIndex()==NONE){
		return b2Transform_inf;
	}
    if (NULL!=custom_start){
        _start=*custom_start;
    }
	return b2MulT(_start, d->pose());
}

EndCriteria Task::getEndCriteria(const Disturbance &d){
	EndCriteria result;
	switch(disturbance.getAffIndex()){
	case PURSUE:{
		result.angle=Angle(0);
		result.distance = Distance(0+DISTANCE_ERROR_TOLERANCE);
	}
	break;
	default:
	result.distance = BOX2DRANGE;
	break;
}
return result;
}

bool Task::endCriteria_met(Angle & a, Distance & d){
	bool result=false;
	Angle approxEndAngle(endCriteria.angle.get()+M_PI_4/HZ);
	if (action.getOmega()==0){
		approxEndAngle.setValid(false);
	}
	switch (affordance){
		case PURSUE:
			result= d<=endCriteria.distance && a<approxEndAngle; 
			break;
		default:
			result= d>=endCriteria.distance && a>=endCriteria.angle; 
			break;
	}
	return result;
}

bool Task::isTurnFinished(const b2Transform & robotTransform, Direction dir){
	float safeAngle=SAFE_ANGLE;
	if (getAffIndex()==AVOID){
		safeAngle=endCriteria.angle.get();
	}				
	float angleL = start.q.GetAngle()+safeAngle, angleR = start.q.GetAngle()-safeAngle;
	float robotAngle=robotTransform.q.GetAngle();
	int mult= start.q.GetAngle()/(3*M_PI_4);
	if (mult>0){
		if (dir==LEFT& robotAngle<0){
			robotAngle+=2*M_PI;
		}
	}
	else if (mult<0){
		if (dir==RIGHT & robotAngle>0){
			robotAngle-=2*M_PI;
		}
	}
	bool finishedLeft=(round(robotAngle*100)/100)>=(round(angleL*100)/100);//-(action.getOmega()*HZ)/2;
	bool finishedRight=(round(robotAngle*100)/100)<=(round(angleR*100)/100);//+(action.getOmega()*HZ)/2;
	return finishedLeft || finishedRight;
}

bool Task::isMoving(){
	return action.getLWheelSpeed()!=0 || action.getRWheelSpeed()!=0;
}