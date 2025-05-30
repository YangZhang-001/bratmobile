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

Threshold Threshold::operator+(const Threshold & t)const{
    Threshold result=*this;
    result.set_Di(result.Di+t.for_Di());
    result.set_Dn(result.Di+t.for_Di());
    return result;
}


Threshold ThresholdLearner::get_threshold(){
    return reflex+get_weighted(reflex);
}



Threshold ThresholdLearner::get_weighted(const Threshold & t){
    Threshold result;
    result.set_Di(t.for_Di()*Di_weights);
    result.set_Dn(t.for_Dn()*Dn_weights);
    return result;
}

float ICO_Learner::learning_rule(float x, float dx){
    return mu*x*dx;
}    

void ICO_Learner::update_bundle(const Bundle & dx, const Bundle & x, BUNDLE_FLAG f){
    Bundle *w=NULL;
    if (f==DI_FLAG){
        w=&Di_weights;
    }
    else if (f==DN_FLAG){
        w=&Dn_weights;
    }
    else{
        return;
    }
    w->add_dx(learning_rule(x.get_x(), dx.get_x()));
    w->add_dy(learning_rule(x.get_y(), dx.get_y()));
    w->add_dangle(learning_rule(x.get_angle(), dx.get_angle()));
    w->add_dwidth(learning_rule(x.get_width(), dx.get_width()));
    w->add_dlength(learning_rule(x.get_length(), dx.get_length()));
}



