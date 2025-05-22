#include "../callbacks.h"

//argv: 1. directory to open 2. timeoff (0=timeron) 3. planning on 4. debug on

Disturbance set_target(int& run, b2Transform start){

}

void forget(Configurator* c){}

int main(int argc, char** argv) {
//HOW MANY FILES IN DIRECTORY
    DIR *dp;
    int i = 0;
    struct dirent *ep;     
    dp = opendir(argv[1]);

    if (dp != NULL && argv[1]!="empty")
    {
        while (ep = readdir(dp)){
            i++;
        }
        closedir(dp);
    }
    else{
        printf("Couldn't open the directory %s", argv[1]);
    }
    char filePrefix[5];
    sprintf(filePrefix, "map");
    int fileCount =1;
    char file[256];
    FILE *f;
    while ( sprintf(file, "%s%s%04i.dat", argv[1], filePrefix, fileCount)>-1 &&(f = fopen(file, "r"))!=NULL){   
        fileCount++;
        memset(file, 0, sizeof(file));
        fclose(f);
    }
    printf("\n%i files\n", fileCount);

    //DATA INTERCFACE

    bool RT=atoi(argv[2]);
	Disturbance target(PURSUE, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
    Configurator configurator(controlGoal);
    LIDAR_In ci;
    Motor_Out m;
    ClosedLoop_Tracker tracker(&goal);
    Wise_Controller wc;
    configurator.register_tracker(&tracker);
    configurator.register_controller(&wc);
    configurator.registerInterface(&ci, &m);
    DataInterface dataInterface(&ci); 
    if (argc>1 && argv[1]){
        dataInterface.folder = argv[1];
    }
    StepCallback cb(&m);
    if (RT){
        TimerDI lidar(dataInterface);
        TimerStep motors(cb);
        lidar.startms(200);
        motors.startms(100);
        configurator.start();
        do {
        }while (getchar());
        lidar.stop();
        motors.stop();
        configurator.stop();
    }
    else if (!RT){
        while  (dataInterface.newScanAvail()){
		if (configurator.ci->isReady()){
			configurator.ci->setReady(false);
			configurator.data2fp= CoordinateContainer(configurator.ci->data2fp);
			configurator.Spawner();
			configurator.get_tracker()->track();
		}
		if (( configurator.getTask()->change& configurator.transitionSystem[configurator.currentVertex].direction!=STOP && configurator.plan.empty() && configurator.getIteration()>1)){
			configurator.goal_changer->change_goal(&configurator.controlGoal);
		}		
		configurator.change_task();

        }
        configurator.running=0;
    }



}