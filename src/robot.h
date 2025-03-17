#ifndef ROBOT_H
#define ROBOT_H
//box2d robot body and kinematic model
#include "box2d/box2d.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include<vector>
#include <map>
#include "const.h"
#include <opencv2/core.hpp>



class Robot {
private: 
	b2FixtureDef fixtureDef;
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		//body->GetUserData().pointer = reinterpret_cast<uintptr_t>(this);
		body->GetUserData().pointer=reinterpret_cast<uintptr_t>(ROBOT_FLAG);
		b2Vec2 center(ROBOT_BOX_OFFSET_X, ROBOT_BOX_OFFSET_Y);
		b2PolygonShape box;
		box.SetAsBox(ROBOT_HALFWIDTH, ROBOT_HALFLENGTH, center, ROBOT_BOX_OFFSET_ANGLE);
		fixtureDef.shape = &box;
		fixtureDef.friction =0;
		body->CreateFixture(&fixtureDef);
		
	}

	static std::vector <b2Vec2> get_vertices(){ //returns vertices in local frame
		std::vector <b2Vec2>result ={b2Vec2(-ROBOT_HALFWIDTH, -ROBOT_HALFLENGTH), b2Vec2(ROBOT_HALFWIDTH, -ROBOT_HALFLENGTH), b2Vec2(-ROBOT_HALFWIDTH, ROBOT_HALFLENGTH), b2Vec2(ROBOT_HALFWIDTH, ROBOT_HALFLENGTH) };
		return result;
	}

	// static cv::RotatedRect get_rect(b2Transform* t=NULL){
	// 	b2Transform * _t;
	// 	if (t!=NULL){
	// 		*_t=*t;
	// 	}
	// 	else{
	// 		*_t=b2Transform_zero;
	// 	}
	// 	cv::RotatedRect rect;
	// 	rect.size.width=2*ROBOT_HALFWIDTH; //should be x axis, in opencv they are swaped idk why
	// 	rect.size.height=2*ROBOT_HALFLENGTH; //should be y axis
	// 	rect.center=cv::Point2f(_t->p.x+ROBOT_BOX_OFFSET_X, _t->p.y+ROBOT_BOX_OFFSET_Y);
	// 	rect.angle=_t->q.GetAngle();
	// 	return rect;
	// }


};



#endif

