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
	object.v1_x(1); //tr
    object.v1_y(0.09);
    object.v2_x(1); //br
    object.v2_y(-0.09);
    object.v3_x(-0.135-0.045); //bl
    object.v3_y(-0.09);
    object.v4_x(-0.135-0.045); //tl
    object.v4_y(0.09);

	if (mypub.publish(object))
	{
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