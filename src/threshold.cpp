#include "threshold.h"

Bundle Bundle::operator*(const Bundle & b)const{
    Bundle result=*this;
    result.x*=b.x;
    result.y*=b.y;
    result.angle*=b.angle;
    result.width*=b.width;
    result.length*=b.length;
    return result;
}

Bundle Bundle::operator*(float f)const{
    Bundle result=*this;
    result.x*=f;
    result.y*=f;
    result.angle*=f;
    result.width*=f;
    result.length*=f;
    return result;
}


bool Bundle::operator<(const Bundle & bf){
    return bf.radius()<radius() && bf.get_angle()<angle && bf.get_width()<width && bf.get_length()<length;
}

Bundle Bundle::operator+(const Bundle & b)const{
    Bundle result=*this;
    result.x+=b.get_x();
    result.y+=b.get_y();
    result.angle+=b.get_angle();
    result.width+=b.get_width();
    result.length+=b.get_length();
    return result;
}

Bundle Bundle::operator-(const Bundle & b)const{
    Bundle result=*this;
    result.x-=b.get_x();
    result.y-=b.get_y();
    result.angle-=b.get_angle();
    result.width-=b.get_width();
    result.length-=b.get_length();
    return result;
}

Bundle linear_rectify(const Bundle & b){
    float x=linear_rectify(b.get_x());
    float y=linear_rectify(b.get_y());
    float angle=linear_rectify(b.get_angle());
    float w=linear_rectify(b.get_width());
    float l=linear_rectify(b.get_length());
    return Bundle(x, y, angle, w, l);
    
}

Threshold ThresholdLearner::get_weighted(const Threshold & t){
    Threshold result;
    result.set_Di(t.for_Di()*Di_weights);
    result.set_Dn(t.for_Dn()*Dn_weights);
    return result;
}

float FF_Learner::learning_rule(float x, float dx){
    return mu*x*dx;
}    

void FF_Learner::update_bundle(const Bundle & error, const Bundle & x, Bundle * w){
    if (w==NULL){
        return;
    }
    w->add_dx(learning_rule(x.get_x(), error.get_x()));
    w->add_dy(learning_rule(x.get_y(), error.get_y()));
    w->add_dangle(learning_rule(x.get_angle(), error.get_angle()));
    w->add_dwidth(learning_rule(x.get_width(), error.get_width()));
    w->add_dlength(learning_rule(x.get_length(), error.get_length()));
}



