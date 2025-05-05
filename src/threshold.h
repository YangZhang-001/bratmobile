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
    Bundle operator*(const Bundle & b);

    /**
     * @brief Multiply bundle for a scalar      
    */
    Bundle operator*(float);

    bool operator<(const Bundle & bf);

    Bundle operator+(const Bundle & b);

    Bundle operator-(const Bundle & b);

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

/**
*Error threshold used to match states or components of states
/*!
Essentially uses distance calculations and (adaptive) thresholding
*/
class Threshold{
    public:

    Threshold()=default;

    Threshold(float e, float a, float d, float aff, float d_dim): 
    endPosition(e), angle(a), dPosition(d), affordance(aff), D_dimensions(d_dim){}

    float for_robot_position(){
        return endPosition;
    }

    float for_robot_angle(){
        return angle;
    }

    float for_affordance(){
        return affordance;
    }

    /** 
     * @brief Returns a bundle of thresholds for the initial disturbance
    */
    Bundle for_Di(){ 
        Bundle result(dPosition, dPosition, angle, D_dimensions, D_dimensions);
        return result*Di_weights;
    }

    /** 
     * @brief Returns a bundle of thresholds for the initial disturbance
    */
    Bundle for_Dn(){ 
        Bundle result(dPosition, dPosition, angle, D_dimensions, D_dimensions);
        return result*Dn_weights;
    }

    /**
     * @brief Returns a bundle of parameters for disturbance matching
     */
    Bundle bundle_threshold(){
        return Bundle(dPosition, dPosition, angle, D_dimensions, D_dimensions);
    }

    /**
     * @brief Tune weights for Di
     * 
     * @param error the error
     */
    void Di_tune(const Bundle & error);

    void log(){
        FILE *f= fopen("/tmp/threshold.txt", "a");
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
        FILE *f= fopen("/tmp/threshold.txt", "w");
        fclose(f);
    }

    void set_learning_rate(float f){
        mu=f;
    }

    private:
        float endPosition=0.05;// maximum radius from candidate state's end pose
        float angle= M_PI/6; // maximum angle difference
        float dPosition= 0.065;// maximum difference between disturbance positions
        float affordance =0; //maximum difference between affordances
        float D_dimensions=0.03; //maximum differences in disturbance dimensions
        Bundle Di_weights, Dn_weights;
        float mu=0.001; //learning rate
};
