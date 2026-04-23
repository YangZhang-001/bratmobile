#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>

#include "targetloc.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    ~Window() {
        targetLoc.stop();
    }

private:
    QHBoxLayout  *h1Layout;
    QHBoxLayout  *h2Layout;
    QVBoxLayout  *vLayout;
    QLabel       *imageL;
    QLabel       *imageR;
    QLabel       *imageCombined;
    QLabel       *imageDisparity;

    // blends the current L and R images and displays it
    void blendLRandDisplayD();

    void timerEvent(QTimerEvent *event);

    TargetLoc targetLoc;

};

#endif // WINDOW_H
