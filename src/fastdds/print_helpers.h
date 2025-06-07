#ifndef PRINT_HELPERS_H
#define PRINT_HELPERS_H

#include <iostream>

void print_xy(auto & x, auto & y){
    std::cout << "x: " << x << "y: " <<y;
}

void print_bounds(auto &x_low, auto & y_low, auto& x_high, auto& y_high){
    std::cout << "lower bound: ";
    print_xy(x_low, y_low);
    std::cout <<"  upper bound: ";
    print_xy(x_high, y_high);
    std::cout<<std::endl;
}


#endif