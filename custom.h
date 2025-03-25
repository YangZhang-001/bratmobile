#include "configurator.h"
//#include "libcam2opencv.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
//#include "Iir.h"
//#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES

std::mutex ctr_mutex;

void get_Foldername(char* custom, char name[60]){
    time_t now =time(0);
	tm *ltm = localtime(&now);
	int y,m,d, h, min;
	y=ltm->tm_year-100;
	m = ltm->tm_mon +1;
	d=ltm->tm_mday;
	h= ltm->tm_hour;
	min = ltm->tm_min;
	sprintf(name, "%s_%02i%02i%02i_%02i%02i",custom, d,m,y,h,min);
}


Disturbance set_target(int&, b2Transform);

std::vector <BodyFeatures> WorldBuilder::processData(const CoordinateContainer& points, const b2Transform& start){
    std::vector <BodyFeatures> result;
    std::vector <Pointf> ptset= set2vec(points);
    std::pair<bool,BodyFeatures> feature= bounding_box(ptset);
    if (feature.first){
        feature.second.pose.q.Set(start.q.GetAngle());
        result.push_back(feature.second);
    }
    return result;
}


class LidarInterface : public A1Lidar::DataInterface{
LIDAR_In * ci;
public: 
    int mapCount =0;

    LidarInterface(LIDAR_In * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		if (ci == NULL){
			printf("null pointer to ci\n");
			return;
		}
		//ci->data.clear();
    	ci->setReady(false);
		ci->data2fp.clear();
		mapCount++;
		Pointf p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s\n", name);
		if (ci->debugOn){
			f=fopen(name, "w");
		}
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				float x2 = round(data.x*100)/100; //resolution adjus
				float y2 = round(data.y*100)/100;
				p2f=Pointf(x2, y2);
				ci->data2fp.insert(p2f);
				if (ci->debugOn){
					fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
				}
            }
		}
		if (ci->debugOn){
		fclose(f);
		}
		ci->setReady(1);
		ci->iteration++;

	}


};

class MotorCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    float L=0;
	float R=0;
public:
int ogStep=0;
Motor_IO * mio;
int run=0;

MotorCallback(Motor_IO *_mio): mio(_mio){
}
void step( AlphaBot &motors){
	if (c->getIteration() <=0){
		return;
	}
	if (!mio->running){
		motors.setRightWheelSpeed(0);
 	    motors.setLeftWheelSpeed(0);		
	}
	//ctr_mutex.lock();
	//printf("graph size=%i\n", c->transitionSystem.m_vertices.size());
	mio->setReady(false);
	mio->track_task_execution();
	//printf("tracked\n");
	EndedResult er = mio->goal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, false);
	if (er.ended && mio->task.change){ //|| (er2.ended & c->getTask()->motorStep<1 & c->planVertices.empty())
		run++;
		Disturbance new_goal=set_target(run, mio->goal.start);
		mio->goal = Task(new_goal, UNDEFINED);
		printf("setting new goal");
		//c->transitionSystem[c->movingVertex].Di=new_goal;
		if (c->is_benchmarking()){
			FILE * f = fopen(statFile, "a+");
			fprintf(f, "!");
			fclose(f);			
		}
	}
	mio->change_task(mio->task.change,  mio->plan,c->transitionSystem, c->controlGoal, *c->getTask(), c->currentVertex);
	printf("changed\n");
	R= mio->task.getAction().getRWheelSpeed();
	L=mio->task.getAction().getLWheelSpeed(); //*1.05
	if (mio->task.direction==LEFT){
		R*=1.37; //23
		L*=1.37;
	}
	else if (mio->task.direction==RIGHT){
		R*=1.07; //17
		L*=1.07;
	}
	else if (mio->task.direction==DEFAULT){
		R*=1.15*1.1;
		L*=1.15;
	}
	mio->setReady(true);
	//ctr_mutex.unlock();	
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
	printf(",R=%f\tL=%f\n",c->getTask()->getAction().getRWheelSpeed(), c->getTask()->getAction().getLWheelSpeed());
}
};


