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

#include "ObjectPackagePubSubTypes.h"
#include "print_helpers.h"


#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include "CppTimer.h"

using namespace eprosima::fastdds::dds;

#ifndef PUBLISHER_CPP
#define PUBLISHER_CPP

class ObjectPackagePublisher
{
private:

    DomainParticipant* participant_ = nullptr;

    Publisher* publisher_ = nullptr;

    Topic* topic_ = nullptr;

    DataWriter* writer_ = nullptr;

    TypeSupport type_;

    class PubListener : public DataWriterListener
    {
    public:

        PubListener()
            : matched_(0)
        {
        }

        ~PubListener() override
        {
        }

        void on_publication_matched(
                DataWriter*,
                const PublicationMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                        << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
            }
        }

        std::atomic_int matched_;

    } listener_;

public:

    ObjectPackagePublisher() : type_(new ObjectPackagePubSubType()) {}

    virtual ~ObjectPackagePublisher()
    {
        if (writer_ != nullptr)
        {
            publisher_->delete_datawriter(writer_);
        }
        if (publisher_ != nullptr)
        {
            participant_->delete_publisher(publisher_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the publisher
    bool init()
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the publications Topic
	// !! Important that this matches with the name of message defined in ObjectPackage.idl !!
        topic_ = participant_->create_topic("ObjectPackageTopic", "ObjectPackage", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Publisher
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        // Create the DataWriter
        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }
        return true;
    }

    //!Send a publication
    bool publish(ObjectPackage& object)
    {
        if (listener_.matched_ > 0)
        {
            writer_->write(&object);
            return true;
        }
        return false;
    }

};


class EventEmitter : public CppTimer {

public:
    ObjectPackagePublisher mypub;
    uint32_t samples_sent = 1;
    
    void timerEvent() {
	ObjectPackage object;
    //robot init
	object.robot_high_x(0.135-0.045);
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
#endif

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