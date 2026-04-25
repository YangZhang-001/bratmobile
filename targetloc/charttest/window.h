#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();

    void timerEvent(QTimerEvent *);


private:
    QVBoxLayout  *vLayout;
    QChart *chart;
    QChartView *chartView;
    QScatterSeries *series;
    float t = 0;
};

#endif // WINDOW_H
