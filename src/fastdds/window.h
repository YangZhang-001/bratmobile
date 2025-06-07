#ifndef QT_WINDOW_H
#define QT_WINDOW_H
#include "subscriber.h"
//#include <qwt/qwt_thermo.h>
#include <qwt/qwt_plot.h>
//#include <qwt/qwt_plot_curve.h>

#include <QBoxLayout>
//#include <QPushButton>

class DDSQtListener:public DataReaderListener{
    public:
    virtual void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info);
    void on_data_available(DataReader* reader);

};

class Window : public QWidget, DataReaderListener{
    Q_OBJECT
    ObjectPackage object;
    QVBoxLayout  *vLayout=nullptr;  // vertical layout
    QHBoxLayout  *hLayout=nullptr;  // horizontal layout
    QwtPlot * plot=nullptr;
    QPainter * painter=nullptr;

    public:
    Window(); // default constructor - called when a Window is declared without arguments
    ~Window();

    virtual void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info);
    void on_data_available(DataReader* reader);



//useful==drawrect!

   // void timerEvent(QTimerEvent *);
};

#endif