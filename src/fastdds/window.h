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
const qreal tl_x=0.09;
const qreal tl_y=0.09;
const qreal br_x=-0.18;
const qreal br_y=-0.09;

class RenderArea: public QWidget{
    Q_OBJECT
    //explicit RenderArea(QWidget *parent = nullptr);
    protected:

    void paintEvent(QPaintEvent * event) override;
};


//#include <QPushButton>
class Window : public QWidget, public DataReaderListener{
    Q_OBJECT
    ObjectPackage object;
    // QVBoxLayout  *vLayout=nullptr;  // vertical layout
    // QHBoxLayout  *hLayout=nullptr;  // horizontal layout
    // QwtPlot * plot=nullptr;
    QPainter * painter=new QPainter(this);
    ObjectPackageSubscriber subscriber;
    QRectF robot=QRectF(0.09, 0.09, 0.0135*2, 0.09*2);

    //QPointF tl(tl_x, tl_y), br(br_x, br_y);
    public:
    Window(); // default constructor - called when a Window is declared without arguments
    ~Window();

    virtual void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info);
    void on_data_available(DataReader* reader);

    void paintEvent(QPaintEvent *)override;
    /**
     * @brief starts subscriber acquisition from publisher
     * 
     */
    void start();

//useful==drawrect!

   // void timerEvent(QTimerEvent *);
};




#endif