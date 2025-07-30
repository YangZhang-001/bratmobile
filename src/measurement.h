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
    float getStandardError(Measurement, float); //relative standard error

};



class Angle: public Measurement{
    public:
    Angle(){}
    Angle(float f)
    {   value =round(f*1000)/1000;
        valid =1;}
};

class Distance: public Measurement{
    public:
    Distance(){}
    Distance(float f)
    {value = round(f*1000)/1000;
    valid =1;}
};

struct EndCriteria{
    friend Task;
    Angle angle;
    Distance distance;    //max distance, ideal
    //bool outOfSight=true;
    //bool valid_d=false;
    float getStandardError(Angle, Distance);
    float getStandardError(Angle, Distance, State);
    bool hasEnd();


    void operator=( EndCriteria ec){
        angle=ec.angle;
        distance=ec.distance;
    }
    //protected:
    void adjust(const b2Transform&);


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