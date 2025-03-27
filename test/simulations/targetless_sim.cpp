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

    if (dp != NULL)
    {
        while (ep = readdir(dp)){
            i++;
        }
        closedir(dp);
    }
    else{
        printf("Couldn't open the directory%s", argv[1]);
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
    Task controlGoal;
    Configurator configurator(controlGoal);
    LIDAR_In ci;
    Motor_Out m;
    if (argc>3){
		configurator.setSimulationStep(atof(argv[3]));
    }
    configurator.registerInterface(&ci, &m);
    DataInterface dataInterface(&ci); 
    dataInterface.folder = argv[1];
    StepCallback cb(&m);

    if (RT){
        TimerDI lidar(dataInterface);
        TimerStep motors(cb);
        lidar.startms(200);
        motors.startms(100);
        configurator.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200*fileCount)); //simulates lidar
        lidar.stop();
        motors.stop();
        configurator.stop();
    }
    else if (!RT){
        while  (dataInterface.newScanAvail()){
            configurator.running=1;           
            configurator.run(&configurator);
        }
        configurator.running=0;
    }



}