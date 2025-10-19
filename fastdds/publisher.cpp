// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright 2025 Bernd Porr
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file ObjectPackagePublisher.cpp
 *
 */

#include "print_helpers.h"
#include "publisher.h"


class EventEmitter : public CppTimer {

public:
    ObjectPackagePublisher mypub;
    uint32_t samples_sent = 1;
    
    void timerEvent() {
	ObjectPackage object;
    //robot init
	object.robot_high_x(1);
    object.robot_high_y(0.09);
    object.robot_low_x(-0.135-0.045);
    object.robot_low_y(-0.09);
    //Di init (fake)
    object.Di_high_x(0.45);
    object.Di_high_y(-0.05);
    object.Di_low_x(0.40);
    object.Di_low_y(0.05);
    //Di init (fake)
    object.goal_high_x(1.01);
    object.goal_high_y(0.01);
    object.goal_low_x(1.00);
    object.goal_low_y(0.0);
	if (mypub.publish(object))
	{
        std::cout <<"Package with Robot ";
        print_bounds(object.robot_low_x(), object.robot_low_y(), object.robot_high_x(), object.robot_high_y());
        std::cout<<std::endl << "Di ";
        print_bounds(object.Di_low_x(), object.Di_low_y(), object.Di_high_x(), object.Di_high_y());
        std::cout<<std::endl << "goal ";
        print_bounds(object.goal_low_x(), object.goal_low_y(), object.goal_high_x(), object.goal_high_y());
	    std::cout << " SENT" << std::endl;
	    samples_sent++;
	} else {
	    std::cout << "No messages sent as there is no listener." << std::endl;
	}
    }

    void start() {
	std::cout << "Starting publisher." << std::endl;

	if(!mypub.init())
	{
	    std::cerr << "Pub not init'd." << std::endl;
	    return;
	}
        startms(1000);
    }

};

int main(
    int,
    char**)
{
    EventEmitter ee;
    ee.start();
    getchar();
    //std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    ee.stop();
}