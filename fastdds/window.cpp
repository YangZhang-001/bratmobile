#include "window.h"


void RobotReaderListener::on_data_available(DataReader* reader){
        SampleInfo info;
        ObjectPackage object;
        if (reader->take_next_sample(&object, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {   
                //access     
                unpacked.set(object);
                notify();

            }
        }
    }

void RobotReaderListener::notify(){
    emit newObject(unpacked);
}


void RobotReaderListener::on_subscription_matched(
    DataReader* reader,
    const SubscriptionMatchedStatus& info)        {
        if (reader==NULL){
            std::cout<<"null reader!"<<std::endl;
        }
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
    

void Window::paintEvent(QPaintEvent *){
   // std::cout<<"painting event!"<<std::endl;
    QPainter painter(this);
    painter.setWindow(logical_rect);
    painter.setPen(QPen());
    painter.drawPoint(point);
    painter.setPen(QColor("red"));
    painter.drawPolygon(goal);
    painter.setPen(QColor("green"));
    painter.drawPolygon(attention);
    painter.setPen(QColor("blue"));
    painter.drawRect(robot);
    painter.setPen(QColor("black"));
    painter.drawPolygon(Di);
}

Window::Window(){
    update();
    setGeometry(m_geometry);
}



void Window::start(){
    // for (auto subscriber: subscribers){
    //     if (subscriber==NULL){
    //         std::cerr<<"no subscriber!"<<std::endl;
    //         return;
    //     }
    //     subscriber->registerListener(this);
    //     if(!subscriber->init())
    //     {
    //     std::cerr << "Could not init the subscriber." << std::endl;
    //     }        
    // }

}

