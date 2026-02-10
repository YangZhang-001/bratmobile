#ifndef QT_WINDOW_H
#define QT_WINDOW_H
#include "subscriber.h"
//#include <qwt/qwt_thermo.h>
//#include <qwt/qwt_plot.h>
//#include "../const.h"
//#include <qwt/qwt_plot_curve.h>
#include <QWidget>
#include <QBoxLayout>
#include <QRectF>
#include <QPainter>
#include "topics.h"

const int scale=100;

/**
 * @brief Unpacks object composed of fundamental data types and groups the data into Qt objects for painting
 * 
 */
class UnpackedObject{
    QPolygon m_object; //should be QPolygon

    public:

    void set(ObjectPackage object){
        m_object.clear();
        m_object<<QPoint(object.v1_x()* scale, object.v1_y()*scale)
                <<QPoint(object.v2_x()*scale, object.v2_y()*scale)
                <<QPoint(object.v3_x()*scale, object.v3_y()*scale)
                <<QPoint(object.v4_x()*scale, object.v4_y()*scale);
    }

    QPolygon getPoly(){
        return m_object;
    }

};

// class RobotSubscriber:public ObjectPackageSubscriber{

// };



class RobotReaderListener:public QObject,public DataReaderListener{
    Q_OBJECT
    virtual void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info)override;
    UnpackedObject unpacked;

    public:

    explicit RobotReaderListener(QObject * parent=nullptr):QObject(parent){}

    void on_data_available(DataReader* reader)override;

    public slots:

    void notify();

    signals:

    void newObject(UnpackedObject _unpacked);

};



class Window : public QWidget{
    Q_OBJECT
    
    QRect m_geometry=QRect(-300, -300, 600, 600); //size of Qtwindow in pixel (bl.x, bl.y, w, l)
    QRect logical_rect=QRect(-120, -120, 240, 240); //world coordinate window
    //std::vector<QtSubscriber*> subscribers; //to one topic!
    QPoint point=QPoint(-50, -50);
    QRectF robot=QRectF(-0.18*scale, -0.09*scale, 0.135*2*scale, 0.09*2*scale);
    QPolygon Di, goal, attention;
    protected:
    void paintEvent(QPaintEvent *)override;

    // void registerSubscriber(QtSubscriber* sub){
    //     subscribers.push_back(sub);
    // }



    public:
    Window(); // default constructor - called when a Window is declared without arguments
    ~Window(){}


    public slots:

    void setDi(UnpackedObject _Di){
        Di=_Di.getPoly();
        update();
    }

    void setGoal(UnpackedObject _goal){
        goal=_goal.getPoly();
        update();
    }

    void setAttention(UnpackedObject _att){
        attention=_att.getPoly();
        update();
    }


};




#endif