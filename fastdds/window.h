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
    ObjectPackageSubscriber subscriber;
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
        QPoint m_goal;
        QRect m_attentionWindow;
        QRect m_Di;

        public:

        void set_goal(float x, float y){
            m_goal.setX(x);
            m_goal.setY(y);
        }

        QPoint goal(){
            return m_goal;
        }

        /**
         * @brief Sets attention window, assumed to always be an upright box
         * 
         */
        void set_attention_window(float hx, float hy, float lx, float ly){
            m_attentionWindow=QRect(QPoint(lx, hy), QPoint(hx, ly));
        }

        QRect attentionWindow(){
            return m_attentionWindow;
        }

        void set_Di(float hx, float hy, float lx, float ly){
            m_Di=QRect(QPoint(lx, hy), QPoint(hx, ly));
        }

        QRect Di(){
            return m_Di;
        }
    }unpacked;

    public:
    Window(); // default constructor - called when a Window is declared without arguments
    ~Window(){}

    virtual void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info);
    void on_data_available(DataReader* reader);

    /**
     * @brief starts subscriber acquisition from publisher
     * 
     */
    void start();


};




#endif