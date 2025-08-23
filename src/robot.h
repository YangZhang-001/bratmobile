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

/**
 * @brief Robot class for Box2D simulation.
 * 
 */
class Robot {
private: 
	b2FixtureDef fixtureDef;
	b2Body* m_body=NULL;
	b2PolygonShape m_box;
	public:


	Robot(b2World * world) {
		b2BodyDef m_bodyDef;
		m_bodyDef.type = b2_dynamicBody;
		m_bodyDef.position.Set(0.0f, 0.0f);
		m_body = world->CreateBody(&m_bodyDef);
		//body->GetUserData().pointer = reinterpret_cast<uintptr_t>(this);
		m_body->GetUserData().pointer=reinterpret_cast<uintptr_t>(ROBOT_FLAG);
		b2Vec2 center(ROBOT_BOX_OFFSET_X, ROBOT_BOX_OFFSET_Y);
		m_box.SetAsBox(ROBOT_HALFWIDTH, ROBOT_HALFLENGTH, center, ROBOT_BOX_OFFSET_ANGLE);
		fixtureDef.shape = &m_box;
		fixtureDef.friction =0;
		m_body->CreateFixture(&fixtureDef);
		
	}

	b2Body* body(){return m_body;} 

	b2PolygonShape box(){return m_box;}
/**
 * @brief Returns vertices in local frame. Order: bl, br, tl, tr 
 */
	static std::vector <b2Vec2> get_vertices(){ 
		std::vector <b2Vec2>result ={b2Vec2(-ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, -ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y), 
									b2Vec2(ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, -ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y), 
									b2Vec2(-ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y), 
									b2Vec2(ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y) };
		return result;
	}

};



#endif

