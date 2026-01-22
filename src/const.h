#ifndef CONST_H
#define CONST_H

#include "box2d/box2d.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
#include<vector>
#include <map>
#include <mutex>


const float SAFE_ANGLE =M_PI_2;
const float MAX_TURN =M_PI;
const float ROBOT_HALFWIDTH =0.135; 
const float ROBOT_HALFLENGTH =0.09; 
const float ROBOT_BOX_OFFSET_X= 0.09-ROBOT_HALFWIDTH;
const float ROBOT_BOX_OFFSET_Y =0;
const float ROBOT_BOX_OFFSET_ANGLE =0;
const float BETWEEN_WHEELS =.15;
const float MAX_SPEED=.2;
const float MAX_OMEGA =2; //2 radians
const float ANGLE_ERROR_TOLERANCE =5 * M_PI/180;
const float BOX2DRANGE =1.0;
const float LIDAR_RANGE =1.01;
const float HZ =10.0;
const float MAX_ANGLE_ERROR= M_PI;
const float SAFE_DISTANCE =sqrt(ROBOT_HALFLENGTH*ROBOT_HALFLENGTH+ROBOT_HALFWIDTH*ROBOT_HALFWIDTH); //DEFAULT backtracking distance
const float MAX_DISTANCE_ERROR =2*BOX2DRANGE;
const float MOTOR_CALLBACK =.1; //MOTOR CALL BACK EVERY .1 s
const float LIDAR_SAMPLING_RATE=0.2;
const float TURN_FRICTION =.86; //.75
const float DISTANCE_ERROR_TOLERANCE=.02; //0.02
const float RELAXED_DIST_ERROR_TOLERANCE=0.06;
const float TRACKING_ERROR_TOLERANCE = MAX_SPEED*MOTOR_CALLBACK*0.5; 
const float TRACKING_ANGLE_TOLERANCE =MAX_OMEGA*0.5*MOTOR_CALLBACK;
const float FORGET_THRESHOLD=0.05;
const float D_POSE_MARGIN=0.065;
const float D_DIMENSIONS_MARGIN=0.03;
const b2Transform b2Transform_zero=b2Transform(b2Vec2_zero, b2Rot(0));
const b2Transform b2Transform_inf=b2Transform(b2Vec2(10000, 10000), b2Rot(MAX_ANGLE_ERROR));
const float ANGLE_RESOLUTION=M_PI/(2*HZ); 
const float MIN_BODY_DIMENSION=0.0005;
const float DEG_TO_RAD_K=0.01745329252;
const float SIM_DURATION = int(BOX2DRANGE*2 /MAX_SPEED);

//camera filtering
const int POS_IT=2;
const int VEL_IT=6;
const int FPS=30;
const int order=3;
const int DC=0; //HZ
const int cutoff_frequency=4; //HZ
const float band_width=0.5;
const uintptr_t ROBOT_FLAG=0x1, DISTURBANCE_FLAG=0x2;

//enums
enum InnateAffordances {NONE, AVOID, PURSUE}; //for ease of identification, previously also ATTACK and EXPLORE
enum Direction{LEFT, RIGHT, DEFAULT, BACK, STOP, UNDEFINED};

//KINEMATICS

const float WHEEL_SPEED_DEFAULT=0.5f;
const float WHEEL_SPEED_TURN=(M_PI_4*BETWEEN_WHEELS)/(MAX_SPEED*2); //0.353429f; //0.218182f;
 
const std::map <Direction, std::pair<float, float>> default_kinematics={{DEFAULT, std::pair<float, float>(WHEEL_SPEED_DEFAULT*MAX_SPEED, 0)},
                                                                         {LEFT, std::pair<float, float>(0, WHEEL_SPEED_TURN)},
                                                                         {RIGHT, std::pair<float, float>(0, -WHEEL_SPEED_TURN)}};

const b2Vec2 GRAVITY(0,0);

#define PLANNING true
#define BENCHMARKING true
extern bool DEBUG=false;


const std::map<Direction, char*> dirmap={{DEFAULT, "DEFAULT"}, {LEFT, "LEFT"}, {RIGHT, "RIGHT"}, {STOP, "STOP"}, {UNDEFINED, "UNDEFINED"}, {BACK, "BACK"}};


#endif
