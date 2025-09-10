#pragma once
#ifndef PRINT_HELPERS_H
#define PRINT_HELPERS_H

#include <iostream>

inline void print_xy(float & x, float & y){
    std::cout << "x: " << x << "y: " <<y;
}

inline void print_bounds(float &x_low, float & y_low, float& x_high, float& y_high){
    std::cout << "lower bound: ";
    print_xy(x_low, y_low);
    std::cout <<"  upper bound: ";
    print_xy(x_high, y_high);
    std::cout<<std::endl;
}


#endif