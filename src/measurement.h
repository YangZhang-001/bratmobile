#ifndef MEASUREMENT_H
#define MEASUREMENT_H
#include "graphTools.h" 

class Task; //forward decl
/**
 * @brief A class for a single Task execution info measurement
 * 
 */
class Measurement{
protected:
    bool valid =0;
    float value=0;
public:
    Measurement(){}

    bool isValid(){
        return valid;
    }

    float get_signed(){
        return value;
    }

    //returns unsigned value
    float get(){ 
        return fabs(value);
    }

    void set(float f){
        value =f;
    }

    void setValid(bool b){
        valid = b;
    }

        /**
     * @brief Compares absolute values
     */
    bool operator>(Measurement &);
    /**
     * @brief Compares absolute values
     */
    bool operator<(Measurement &);
    /**
     * @brief Compares absolute values
     */
    bool operator<=(Measurement &);
    /**
     * @brief Compares absolute values
     */
    bool operator>=(Measurement &);

    /**
     * @brief Compares signed values
     */
    bool operator==(Measurement& m2);

    /**
     * @brief Gets error between this and another measurement, normalised by the maximum error value error can take
     * 
     * @return float 
     */
    float getStandardError(Measurement, float); //relative standard error

};



class Angle: public Measurement{
    public:
    Angle(){}
    Angle(float f){  
        //value =round(f*1000)/1000;
        value = f; //no rounding
        valid =1;}
};

class Distance: public Measurement{
    public:
    Distance(){}
    Distance(float f)
    {value = round(f*1000)/1000;
    valid =1;}
};

/**
 * @brief Defines the end criteria for a Task, i.e. when it is considered to be finished
 * 
 */
struct EndCriteria{
    Angle angle;
    Distance distance;    //max distance, ideal
    /**
     * @brief Calculates normalised cumulative standard error for the given angle and distance
     */
    float getStandardError(Angle a, Distance d);

    /**
     * @brief Calculates the normalised cumulative standard error for the given angle, distance; state information @param n are used to add a penalty if the state results in collision
     */
    float getStandardError(Angle a, Distance d, State n);
    bool hasEnd();


    void operator=( EndCriteria ec){
        angle=ec.angle;
        distance=ec.distance;
    }

    /**
     * @brief Ajusts endcriteria based on the delta transform (2D transform representing how much the robot has travelled)
     * 
     * @param delta 
     */
    void adjust(const b2Transform& delta);


};

/**
 * @brief Provides information on whether a Task has ended and with what heuristic -estimated- cost (chi) and past cost (gamma)
 * 
 */
struct EndedResult{
	bool ended=0;
	float estimatedCost=0; //dot product of end criteria, heuristic cost
    float cost=0; //gamma

    EndedResult() = default;

};

float SignedVectorLength(b2Vec2);


#endif