#include "window.h"


void Window::on_data_available(DataReader* reader){
        SampleInfo info;
        ObjectPackage object;
        if (reader->take_next_sample(&object, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {   
                unpacked.set_goal(object.goal_low_x()*scale, object.goal_low_y()*scale);
                unpacked.set_attention_window(object.robot_high_x()*scale, object.robot_high_y()*scale, object.robot_low_x()*scale, object.robot_low_y()*scale);
                update();
                //plot->replot();
                //paintEvent(NULL);
            }
        }
    }
    

void Window::paintEvent(QPaintEvent *){
    std::cout<<"painting event!"<<std::endl;
    QPainter painter(this);
    painter.setWindow(logical_rect);
    painter.setPen(QPen());
    painter.drawPoint(point);
    painter.setPen(QColor("red"));
    painter.drawPoint(unpacked.goal());
    painter.setPen(QColor("green"));
    painter.drawRect(unpacked.attentionWindow());
    painter.setPen(QColor("cyan"));
    painter.drawRect(robot);
}

Window::Window(){
    update();
    subscriber.registerListener(this);
    setGeometry(m_geometry);
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
    // painter->begin(this);
}
