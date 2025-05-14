#include "../test_essentials.h"

/**
 * @brief Plain feedforward: error in one dimension only correlated with the input in that dimension
 * 
 */
class FF_Learner: public ThresholdLearner{

    float learning_rule(float x, float dx){
        return mu*x*dx;
    }    

    void update_bundle(const Bundle & error, const Bundle & x, Bundle * w){
        if (w==NULL){
            return;
        }
        w->add_dx(learning_rule(x.get_x(), error.get_x()));
        w->add_dy(learning_rule(x.get_y(), error.get_y()));
        w->add_dangle(learning_rule(x.get_angle(), error.get_angle()));
        w->add_dwidth(learning_rule(x.get_width(), error.get_width()));
        w->add_dlength(learning_rule(x.get_length(), error.get_length()));
    }
};

int main(){

}