#include "custom_robot.h"
#include <c1lidarrpi.h>

#ifdef BRAT_BUILD_TARGETLOC_NAVIGATION

// ncurses also uses OK, which conflicts with OpenCV stitching
#ifdef OK
#undef OK
#endif

#include "targetloc.h"
#include "targetlocreceiver.h"
#include <chrono>
#include <cmath>
#include <exception>
#endif

const bool DEBUG=false;

#ifdef BRAT_BUILD_TARGETLOC_NAVIGATION
constexpr float NAVIGATION_TARGET_MIN_RANGE_M = 0.20F;
#endif

int main(int argc, char** argv) {

#ifdef BRAT_BUILD_TARGETLOC_NAVIGATION
	std::cout<<"Waiting for a stable TargetLoc result"<<std::endl;
#else
	std::cout<<"Navigating to Target with Brat2"<<std::endl;
#endif

	C1Lidar lidar;

#ifdef BRAT_BUILD_TARGETLOC_NAVIGATION

	TargetLoc targetLoc;

	//require three close target results
	StableTargetLocReceiver targetReceiver(3, 0.03F);

	// navigation is not connected while we are finding the target
	LidarInterface dataInterface(nullptr);

	//compact output is easier to read in the combined test.
	targetLoc.setOutputMode(TargetLoc::OutputMode::Compact);
	targetLoc.registerNewTargetDetectedCallback(&targetReceiver);

	//the first LiDAR scans belong only to TargetLoc
	dataInterface.startTargetAcquisition(&targetLoc);
	lidar.registerInterface(&dataInterface);

	cv::Point2f targetCoordinate;
	bool targetAvailable = false;

	//start the target acquisition phase before waiting for a result
	try{
		targetLoc.start();

#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
	lidar.start(C1Lidar::ROCK5_SERIAL_DEV);
#else
	lidar.start(C1Lidar::RPI_SERIAL_DEV);
#endif

	//do not wait forever when the target cannot be found.
	targetAvailable = targetReceiver.waitForTarget(
		targetCoordinate,
		std::chrono::seconds(60));
	}

	catch (const std::exception &error) {
		std::cerr<<"Target acquisition failed: "
				 <<error.what()<<std::endl;

		//close the acquisition path before returning an error.
		targetReceiver.close();
		dataInterface.beginTransition();
		targetLoc.stop();
		lidar.stop();

		return 1;
	}

	// one locked target is enough for this run.
	targetReceiver.close();

	// stop new scans and wait for the current callback to finish
	dataInterface.beginTransition();

	// camera and detection work can now stop safely.
	targetLoc.stop();

	if (!targetAvailable) {
		std::cerr<<"No stable TargetLoc result received"<<std::endl;

		lidar.stop();
		return 1;
	}

	std::cout<<"Locked target: x="<<targetCoordinate.x
			 <<" m, y="<<targetCoordinate.y<<" m"<<std::endl;

	// reject an unsafe coordinate before navigation is created.
	const float targetRange =
		std::hypot(targetCoordinate.x, targetCoordinate.y);

	if (!std::isfinite(targetCoordinate.x) ||
		!std::isfinite(targetCoordinate.y) ||
		targetCoordinate.x <= 0.0F ||
		targetRange < NAVIGATION_TARGET_MIN_RANGE_M ||
		targetRange > BOX2DRANGE) {

		std::cerr<<"Target rejected: x="<<targetCoordinate.x
			 <<" m, y="<<targetCoordinate.y
			 <<" m, range="<<targetRange<<" m"
			 <<std::endl;

		lidar.stop();
		return 1;
	}

	std::cout<<"Target accepted for navigation: range="
		     <<targetRange<<" m"<<std::endl;

	//replace the old fixed position with the measured one.
	b2Vec2 targetPosition(
		targetCoordinate.x,
		targetCoordinate.y);

#else

	b2Vec2 targetPosition(BOX2DRANGE, 0);

#endif

	Disturbance target(2, targetPosition);
    Task controlGoal(target, DEFAULT);

    FocusedConfigurator configurator;
	configurator.init(controlGoal);
	LaserFocus wb;
	configurator.register_worldBuilder(&wb);
	HorizonStarPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	OpenLoopController wc;
	configurator.register_controller(&wc);
	Logger logger( "brat3-target", "/tmp");
	configurator.register_logger(&logger);
	configurator.setSimulationStep(.27);

	configurator.registerInterface(&tracker);

#ifdef BRAT_BUILD_TARGETLOC_NAVIGATION

	// start the motor backend before navigation receives scans
	// Rock 5 is still using dry-run mode for this test
	tracker.start();

	//the same LiDAR stream now belongs only to navigation.
	dataInterface.startNavigation(&configurator);

	std::cout<<"Navigation ready. Press Enter to stop."<<std::endl;

#else

	LidarInterface dataInterface(&configurator);
	lidar.registerInterface(&dataInterface);
	
	lidar.start(C1Lidar::RPI_SERIAL_DEV);
	tracker.start();
#endif

	getchar();

#ifdef BRAT_BUILD_TARGETLOC_NAVIGATION

	//let the current navigation callback finish before shutdown.
	dataInterface.beginTransition();

#endif

	tracker.stop();
	lidar.stop();
}
	
	
