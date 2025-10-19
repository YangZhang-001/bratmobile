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

class Window : public QWidget, public DataReaderListener{
    Q_OBJECT
    
    QRect m_geometry=QRect(-300, -300, 600, 600); //size of Qtwindow in pixel (bl.x, bl.y, w, l)
    QRect logical_rect=QRect(-102, -102, 204, 204); //world coordinate window
    std::vector<ObjectPackageSubscriber*> subscribers; //to one topic!
    const int scale=100;
    QPoint point=QPoint(0, 0);
    QRectF robot=QRectF(-0.18*scale, -0.09*scale, 0.135*2*scale, 0.09*2*scale);

    protected:
    void paintEvent(QPaintEvent *)override;

    /**
     * @brief Unpacks object composed of fundamental data types and groups the data into Qt objects for painting
     * 
     */
    class UnpackedObject{
        QPolygon m_object; //should be QPolygon

        public:

        void setObject(ObjectPackage object){
            m_object.clear();
            m_object<<QPointF(object.v1_x()* scale, object.v1_y()*scale)
                    <<QPointF(object.v2_x()*scale, object.v2_y()*scale)
                    <<QPointF(object.v3_x()*scale, object.v3_y()*scale)
                    <<QPointF(object.v4_x()*scale, object.v4_y()*scale);
        }

    }unpacked;

    public:
    Window(); // default constructor - called when a Window is declared without arguments
    ~Window(){}

    virtual void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info);
    void on_data_available(DataReader* reader);

    void registerSubscriber(ObjectPackageSubscriber* sub){
        subscribers.push_back(&sub);
    }

    /**
     * @brief starts subscriber acquisition from publisher
     * 
     */
    void start();


};




#endif