#ifndef CALLBACKS_H
#define CALLBACKS_H
#include "CppTimer.h"
#include <thread>
#include "test_essentials.h"
#include <string>


class DataInterface {
int iteration = 0;
Configurator * configurator=NULL;
std::string folder;
public:

    DataInterface(){}

	bool newScanAvail(bool doPlan=true){ //uncomment sections to write x and y to files		
        iteration++;
	char filePath[1024];
        char folderName[1024];
        sprintf(folderName,"%s", folder.c_str());
        configurator->clearData();
        if (folderName != NULL){
            sprintf(filePath, "%smap%04d.dat", folderName, iteration);
            printf("%s\n", filePath);
            FILE *f;
            if (!(f=fopen(filePath, "r"))){
                throw "can't open file!";
                if (iteration>1){
                    iteration=1;
                }
            }
            else {
                fclose(f);
            }
            std::ifstream file(filePath);
            float x, y;
            while (file>>x>>y){
                if (b2Vec2(x, y).Length()<LIDAR_RANGE){
                    x = round(x*100)/100;
                    y = round(y*100)/100;
                        configurator->insertCoordinate(x, y);
                }

            }
            file.close();
        }
        if (doPlan){
            configurator->newScanEvent();
        }

        return true;
	}

    void set_folder(std::string str){
        folder=str;
    }

    int get_iteration(){
        return iteration;}
        
    void set_iteration(int i){
        iteration=i;
    }

    void registerConfigurator(Configurator *i){
        configurator=i;
    }

    bool has_interface(){
        return configurator!=NULL;
    }

    void reset(){
        folder.clear();
    }

    bool hasFolder(){
        return folder.size()>0;
    }
    
    const std::string getFolder() const {
        return folder;
    }
};

class StepCallback:public MotorInterface{

    int ogStep=0;
public:

    StepCallback()=default;

    void step(){
        L=get_L();
        R= get_R();
    }
};

//FOR THREAD DEBUGGING

class TimerStep: public CppTimer{
    StepCallback sc;
    public:
    TimerStep(StepCallback & _step): sc(_step){}
    void timerEvent(){
        sc.step();
    }
};

class TimerDI: public CppTimer{
    DataInterface di;
    public:
    TimerDI(DataInterface & _di): di(_di){}
    void timerEvent(){
        if (!di.newScanAvail()){
            this->stop();
        }
    }
};


 #endif
