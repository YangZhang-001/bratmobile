#include "attentive.h"
//#include "libcam2opencv.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
//#include "Iir.h"
//#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES

/**
 * * * * DEFINITION OF DATA INTERFACES FOR ROBOT SENSORS/MOTORS
 * 				+ SOME DEBUGGING HELPER FUNCTIONS
 *
 */


class LidarInterface : public A1Lidar::DataInterface{
LIDAR_In * ci=NULL;
public:
    int mapCount =0;

    LidarInterface(LIDAR_In * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		if (ci == NULL){
			std::cerr<<"null pointer to ci"<<std::endl;
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
Motor_Out * mio;
int run=0;

MotorCallback(Motor_Out *_mio): mio(_mio){
}
void step( AlphaBot &motors){
	if (mio==NULL){
		throw ("mio null\n");
	}
    motors.setRightWheelSpeed(mio->get_R()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(mio->get_L());
	printf(",R=%f\tL=%f\n",mio->get_R(), mio->get_L());
}
};



