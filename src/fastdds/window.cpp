#include "window.h"

void Window::on_data_available(DataReader* reader){
        SampleInfo info;
        if (reader->take_next_sample(&object, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (!info.valid_data)
            {   
                std::cout <<"invalid data!"<<std::endl;
            }
        }
    }
    
Window::Window(){
    vLayout=new QVBoxLayout;
    hLayout=new QHBoxLayout;
    plot = new QwtPlot;
    painter=new QPainter;
    subscriber.registerListener(this);
}

Window::~Window(){
    delete vLayout;
    delete hLayout;
    delete plot;
    delete painter;
}

void Window::on_subscription_matched(
    DataReader*,
    const SubscriptionMatchedStatus& info)        {
if (info.current_count_change == 1)
{
    std::cout << "Subscriber matched." << std::endl;
}
else if (info.current_count_change == -1)
{
    std::cout << "Subscriber unmatched." << std::endl;
}
else
{
    std::cout << info.current_count_change
            << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
}
}

void Window::start(){
    if(!subscriber.init())
    {
	std::cerr << "Could not init the subscriber." << std::endl;
    }
}
