#ifndef _THRESHOLD_H
#define _THRESHOLD_H

#include <cmath>
#include <vector>
#include <cstdio>

class BodyFeatures;

/**
*Struct for conveniently grouping weights/threshold associated to BodyFeatures (defined in disturbance.h)
*/
class Bundle{
    float x=1;
    float y=1;
    float angle=1;
    float width=1;
    float length=1;

    public:
    Bundle()=default;

    Bundle(float _x, float _y, float _a, float _w, float _l): x(_x), y(_y), angle(_a), width(_w), length(_l){}
    
    /**
      * @brief Dot product between two bundles
      * 
      * @param b the other bundle
      */
    Bundle operator*(const Bundle & b)const;

    /**
     * @brief Multiply bundle for a scalar      
    */
    Bundle operator*(float)const;

    bool operator<(const Bundle & bf);

    Bundle operator+(const Bundle & b)const;

    Bundle operator-(const Bundle & b)const;

    float radius()const {
        return sqrt(pow(x,2)+pow(y,2));
    }

    float get_x()const {
        return x;
    }

    float get_y()const{
        return y;
    }

    float get_angle()const{
        return angle;
    }

    float get_width()const{
        return width;
    }

    float get_length()const{
        return length;
    }


    float sum_squares()const{
        return pow(x, 2) + pow(y, 2)+pow(angle, 2) +pow(width, 2) +pow(length, 2);
    }

    std::vector <float> get_vector()const{
        return {x, y, angle, width, length};
    }
};

template <typename T>
T linear_rectify(T value){
    if (value<0){
        value=0;
    }
    return value;
}


Bundle linear_rectify(const Bundle &);


/**
* Error threshold used to match states or components of states.
* Essentially uses distance calculations and (adaptive) thresholding.
*/
class Threshold{
    public:
    static constexpr float FIXED_ENDPOSE=0.05;// maximum radius from candidate state's end pose
    static constexpr float FIXED_ANGLE= M_PI/6; // maximum angle difference
    static constexpr float FIXED_DISTPOS= 0.065;// maximum difference between disturbance positions
    static constexpr float FIXED_AFFORDANCE =0; //maximum difference between affordances
    static constexpr float FIXED_DIMENSIONS=0.03; //maximum differences in disturbance dimensions

    Threshold()=default;

    Threshold(float e, float a, float d, float aff, float d_dim): 
    endPosition(e), angle(a), dPosition(d), affordance(aff), D_dimensions(d_dim){}

    float for_robot_position()const{
        return endPosition;
    }

    float for_robot_angle()const{
        return angle;
    }

    float for_affordance()const{
        return affordance;
    }

    /** 
     * @brief Returns a bundle of thresholds for the initial disturbance
    */
    Bundle for_Di()const{ 
        return Di;
    }

    /** 
     * @brief Returns a bundle of thresholds for the initial disturbance
    */
    Bundle for_Dn()const{ 
        return Dn;
    }

    void set_Dn(const Bundle & b){
        Dn=b;
    }

    void set_Di(const Bundle & b){
        Di=b;
    }
    /**
     * @brief Returns a bundle of parameters for disturbance matching
     */
    Bundle bundle_threshold(){
        return Bundle(dPosition, dPosition, angle, D_dimensions, D_dimensions);
    }



    private:
        float endPosition=FIXED_ENDPOSE;// maximum radius from candidate state's end pose
        float angle= FIXED_ANGLE; // maximum angle difference
        float dPosition= FIXED_DISTPOS;// maximum difference between disturbance positions
        float affordance =FIXED_AFFORDANCE; //maximum difference between affordances
        float D_dimensions=FIXED_DIMENSIONS; //maximum differences in disturbance dimensions
        Bundle Di=Bundle(dPosition, dPosition, angle, D_dimensions, D_dimensions), Dn=Di;
};

class ThresholdLearner{
    protected:
    Bundle Di_weights, Dn_weights;
    float mu=0.001; //learning rate
    public:
    ThresholdLearner()=default;

    void set_learning_rate(float f){
        mu=f;
    }

    Bundle get_Di_weights(){
        return Di_weights;
    }

    Bundle get_Dn_weights(){
        return Dn_weights;
    }

    void log(){
        FILE *f= fopen("/tmp/threshold_weights.txt", "a+");
        fprintf(f, "%f\t%f\t%f\t%f\t%f\t\t%f\t%f\t%f\t%f\t%f\n", Di_weights.get_x(), 
                                                                Di_weights.get_y(), 
                                                                Di_weights.get_angle(),
                                                                Di_weights.get_length(),
                                                                Di_weights.get_width(),
                                                                Dn_weights.get_x(), 
                                                                Dn_weights.get_y(), 
                                                                Dn_weights.get_angle(),
                                                                Dn_weights.get_length(),
                                                                Dn_weights.get_width()
                                                                );
        fclose(f);
    }

    void make_log(){
        FILE *f= fopen("/tmp/threshold_weights", "w+");
        fclose(f);
    }

    /**
     * @brief Tune weights for Di
     * 
     * @param error the error
     */
    virtual void Di_tune(const Bundle & error, const Bundle & x)=0;

    /**
     * @brief Updates a threshold according to a custom learning rule
     * 
     * @param error 
     * @param x the input
     * @return Bundle 
     */
    virtual Bundle update_bundle(const Bundle & error, const Bundle & x)=0;
    
    Threshold get_weighted(const Threshold & t){
        Threshold result;
        result.set_Di(t.for_Di()*Di_weights);
        result.set_Dn(t.for_Dn()*Dn_weights);
        return result;
    }


};

#endif
