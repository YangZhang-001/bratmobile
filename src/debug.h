#ifndef DEBUG_H
#define DEBUG_H

#include "worldbuilder.h"
#include <fstream>
#include <bits/stdc++.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <dirent.h>


// class Logger{

// 	//char statFile[100];
// 	std::string fileName;
// 	std::ostream file;


// 	public:

// 	Logger(){}

// 	Logger(char * new_folder, char * _dir=NULL){
// 		//char dirName[50];
// 		std::string dirName;
// 		if (_dir==NULL){
// 			//sprintf(dirName, "benchmark");
// 			dirName="benchmark";
// 		}
// 		else{
// 			//sprintf(dirName, _dir);
// 			dirName=_dir;
// 		}
// 		if (!opendir(dirName.c_str())){
// 			mkdir(dirName.c_str(), 0777);
// 		}
// 		//char new_path[60];
// 		//sprintf(new_path, "%s/%s", dirName, new_folder);
// 		std::string new_path=dirName + "/"+new_folder;
// 		if (!opendir(new_path.c_str())){
// 			mkdir(new_path.c_str(), 0777); //""
// 		}
// 		//TODAYS DATE AND TIME
// 		time_t now =time(0);
// 		tm *ltm = localtime(&now);
// 		int y,m,d, h, min;
// 		y=ltm->tm_year-100;
// 		m = ltm->tm_mon +1;
// 		d=ltm->tm_mday;
// 		h= ltm->tm_hour;
// 		min = ltm->tm_min;
// 		fileName=new_path+"/stats"+d+m+y+ "_"+h+min+".txt";
// 		//sprintf(statFile, "%s/stats%02i%02i%02i_%02i%02i.txt",new_path, d,m,y,h,min);
// 		FILE * f = fopen(fileName.c_str(), "w");
// 		fclose(f);
// 	}



// };



namespace debug{
	
// template <class T>
// void graph_file(const int &, const T&,const Disturbance &, std::vector <vertexDescriptor>,const vertexDescriptor&);

template <class T>
void print_graph(const T& g, const Disturbance & goal, std::vector <vertexDescriptor>plan, const vertexDescriptor& c){
     std::stringstream os;
    auto vs=boost::vertices(g);
    for (auto vi=vs.first; vi!=vs.second; vi++){
		auto es=boost::out_edges(*vi, g);
		if (*vi==c){
			os<<"!";
		}
		for (vertexDescriptor vp:plan){
			if (*vi==vp){
				os<<"*";
			}
		}
		os<<*vi<<"-> ";
		for (auto ei=es.first; ei!=es.second; ei++){
			if (*ei!=edgeDescriptor()){
				os<<(*ei).m_target <<"("<<g[(*ei)].probability<<")";
			}
		}
		os<<"\t(x="<<g[*vi].endPose.p.x<<", y= "<<g[*vi].endPose.p.y<<", theta= "<<g[*vi].endPose.q.GetAngle()<<")\n";
	}
    std::cout<<os.str();
}


template <class T>
void graph_file(const int &it, const T &g, const Disturbance &goal, std::vector<vertexDescriptor>&plan, const vertexDescriptor &c)
{
    char fileName[50];
	sprintf(fileName, "/tmp/graph%04i.txt", it);
	FILE * f=fopen(fileName, "w");
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		auto es=boost::out_edges(*vi, g);
		if (*vi==c){
			fprintf(f,"!");
		}
		for (vertexDescriptor vp:plan){
			if (*vi==vp){
				fprintf(f,"*");
			}
		}
		fprintf(f,"%i -> ", (*vi));
		for (auto ei=es.first; ei!=es.second; ei++){

			fprintf(f, "%i (%f) ", (*ei).m_target, g[(*ei)].probability);
		}
		fprintf(f, "\t(x=%.3f, y= %.3f, theta= %.3f)\n", g[*vi].endPose.p.x, g[*vi].endPose.p.y, g[*vi].endPose.q.GetAngle());
	}
	fclose(f);
}
b2Vec2 GetWorldPoints(b2Body*, b2Vec2 );

char * print_pose(const b2Transform& p, char * msg=NULL);

void print_matrix(const cv::Mat &);

std::vector<b2Vec2> GetBodies( b2World*);

void print_state_difference(const StateDifference & sd, vertexDescriptor v, vertexDescriptor v1);

}



#endif