#include <opencv2/video/tracking.hpp>
#include "configurator.h"
#include "../test_essentials.h"
int main(){
    /*
    Kalman filter model:
    a(t)= a(t-1)+da(t)
    x(t)=cos(da (t)) - sin(da (t))+ x(t-1) + dx(t)
    y(t)=cos(da (t)) + sin(da (t))+ y(t-1) +dy(t)
    */

    //measuremet: rw measurement
    int state_dim=6, measurement_dim=6; //state: x y theta (dx dy dtheta); measurement x y theta 
    cv::KalmanFilter kf(state_dim, measurement_dim);
                                            //  offsets
                                            // x  y  th dx dy dth
    kf.transitionMatrix=(cv::Mat_<float>(6,6)<< 1, 0, 0, 1, 0, 0, //rw x (x(t-1)+ dx(t))
                                               0, 1, 0, 0, 1, 0, //rw y (y(t-1)+ dy(t))
                                               0, 0, 1, 0, 0, 1, //rw  theta(theta(t-1)+ dtheta(t))
                                               0, 0, 0, 1, 0, 0, // rate of update x estimate
                                               0, 0, 0, 0, 1, 0, // rate of update y estimate 
                                               0, 0, 0, 0, 0, 1); // rate update theta estimate
                                               

    kf.measurementMatrix=(cv::Mat_<float>(6, 6)<< 1, 0, 0, 0, 0, 0, //shift x
                                                 0, 1, 0, 0, 0, 0,
                                                 0, 0, 1, 0, 0, 0,
                                                 0, 0, 0, 1, 0, 0, //shift x
                                                 0, 0, 0, 0, 1, 0,
                                                 0, 0, 0, 0, 0, 1
                                                 ); //shift y

    //set custom process noise covariance (higher process noise -> faster adaptation, lower measurement noise ->faster adaptation)
    setIdentity(kf.processNoiseCov, cv::Scalar(1));
    
    setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-4));

    setIdentity(kf.errorCovPost, cv::Scalar(1));
    //initialise with known measurements
    kf.statePre=(cv::Mat_<float>(6,1)<<.5,0,0,0.02, 0, 0);
    kf.statePost=kf.statePre;
    cv::Mat predict=kf.predict(); //i think control goes here
    cv::Mat measurement=(cv::Mat_<float>(6, 1)<< 0.5, 0.0, 0.0, 0.029, -0.01, 0.01);
    kf.correct(measurement);
}