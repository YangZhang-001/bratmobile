#include "threshold.h"

Bundle Bundle::operator*(const Bundle & b){
    Bundle result=*this;
    result.x*=b.x;
    result.y*=b.y;
    result.angle*=b.angle;
    result.width*=b.width;
    result.length*=b.length;
    return result;
}

Bundle Bundle::operator*(float f){
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

Bundle Bundle::operator+(const Bundle & b){
    Bundle result=*this;
    result.x+=b.get_x();
    result.y+=b.get_y();
    result.angle+=b.get_angle();
    result.width+=b.get_width();
    result.length+=b.get_length();
    return result;
}

Bundle Bundle::operator-(const Bundle & b){
    Bundle result=*this;
    result.x-=b.get_x();
    result.y-=b.get_y();
    result.angle-=b.get_angle();
    result.width-=b.get_width();
    result.length-=b.get_length();
    return result;
}




