#ifndef TASK_H
#define TASK_H
#include "measurement.h"

//if body has sensor, return the corresponding fixture
b2Fixture * GetSensor(b2Body * body);

//from the bodies in the world, get the disturbance Di for a Task
b2Body * GetDisturbance(b2World *);

//returns true 
bool overlaps(b2Body *, Disturbance *);

bool overlaps(const b2PolygonShape&, Disturbance *, const b2Transform& robot_pose=b2Transform_zero);

//deletes all bodies in the box2d world
void world_cleanup(b2World & _world);

class Task{
    friend class Configurator;
    protected:
    char planFile[250]; //for debug
    bool debug_k=false; //delete this it's for debugging on the bhenchod pi
    bool change =0;
    b2Transform start=b2Transform_zero;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction= DEFAULT;
    int motorStep=0;
    AffordanceIndex affordance=NONE;


public:
struct Action{
private:
    float linearSpeed=WHEEL_SPEED_DEFAULT*2; //used to calculate instantaneous velocity using omega
    float omega=0; //initial angular velocity is 0
    bool valid=0;
    float R=WHEEL_SPEED_DEFAULT;
    float L=WHEEL_SPEED_DEFAULT;
    
    public:


    Action()=default;

    void init(Direction& direction){
        switch (direction){
        case Direction::DEFAULT:
        L=WHEEL_SPEED_DEFAULT;
        R=WHEEL_SPEED_DEFAULT;
        break;
        case Direction::LEFT:
        L=-WHEEL_SPEED_TURN;
        R=WHEEL_SPEED_TURN;
        break;
        case Direction::RIGHT:
        L=WHEEL_SPEED_TURN;//0.2537;
        R=-WHEEL_SPEED_TURN;
        break;
        default:
        direction=DEFAULT;
        L=0;
        R=0;
        break;
    }
    setVelocities(L, R);
    }

void setVelocities(const float & l,const float &r){
    omega = (MAX_SPEED*(r-l)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide
   // recordedOmega = omega;
    linearSpeed = MAX_SPEED*(l+r)/2;
    //recordedSpeed=linearSpeed;
    valid=1;
}

    b2Vec2 getLinearVelocity(const float &dt=1)const{ //dt integrates
        b2Vec2 velocity;
        velocity.x = linearSpeed *cos(omega)*dt;
        velocity.y = linearSpeed *sin(omega)*dt;
        return velocity;
    }

    b2Transform getTransform(const float &dt=1)const{ //dt integrates
    return b2Transform(getLinearVelocity(dt), b2Rot(getOmega(dt)));
}

    float getRWheelSpeed()const{
        return R;
    }

    float getLWheelSpeed()const{
    return L;
    }

    void setRWheelSpeed(float f){
        R=f;
    }

    void setLWheelSpeed(float f){
        L=f;
    }

    bool isValid(){
        return valid;
    }

    float getLinearSpeed(){
        return linearSpeed;
    }

    float getOmega(const float &dt=1)const{
    return omega*dt;
    }
    
    /**
     * @brief Uses kinematic model to calculate angle
     * @param l left wheel speed (normalised)
     * @param r right wheen speed (normalised)     * 
     */

    float getOmega(const float l, const float r, float dt=1)const{
        float result = (MAX_SPEED*(r-l)/BETWEEN_WHEELS)*TURN_FRICTION*dt;
        return result;
    }

    void setOmega(const float &o){
        omega =o;
    }

    void setLinearSpeed(const float & s){
        linearSpeed =s;
    }


};



class Listener : public b2ContactListener {
 // int iteration=1;
    Disturbance * d_ptr;
    std::vector <b2Body*> collisions;
    public:
    Listener(){}
    Listener(Disturbance * _d): d_ptr(_d){}
        void BeginContact(b2Contact * contact) {
        b2Fixture * fixtureA= contact->GetFixtureA();
        b2Fixture * fixtureB= contact->GetFixtureB();
        b2BodyUserData bodyData = fixtureA->GetBody()->GetUserData();
        if (bodyData.pointer==ROBOT_FLAG) { //if fixtureA belongs to robot
            b2Body * other = contact->GetFixtureB()->GetBody();
            if (fixtureA->IsSensor()){
                if (other->GetUserData().pointer==DISTURBANCE_FLAG){
                    d_ptr->validate();
                }
                return;
            }
            if (fixtureB->IsSensor()){
                return;
            }
            collisions.push_back(other);
        }
        bodyData = fixtureB->GetBody()->GetUserData();
        if (bodyData.pointer==ROBOT_FLAG) {//WAS IF BODYDATA.POINTER if fixtureB belongs to robot
            b2Body * other = contact->GetFixtureA()->GetBody();
            if (fixtureB->IsSensor()){
                if (other->GetUserData().pointer==DISTURBANCE_FLAG){
                    d_ptr->validate();
                }
                return;
            }
            if (fixtureA->IsSensor()){
                return;
            }
            collisions.push_back(other);
            }       
        }

        std::vector <b2Body*> get_collisions(){
            return collisions;
        }
	};

// struct Correct{
    
//     Correct(){}

//     void operator()( Action&, int);

//     float errorCalc(Action , double);

//     float getError(){
//         return p();
//     }

//     float Ki(){
//         return ki;
//     }

//     float Kp(){
//         return kp;
//     }
//     float Kd(){
//         return kd;
//     }

//     float get_i(){
//         return i;
//     }

//     float get_d(){
//         return d;
//     }

//     float update(float);

//     void reset(){
//         p_buffer=std::vector <float>(bufferSize,0);
//         i=0;
//         d=0;
//         mf.buffer=std::vector<float>(mf.kernelSize,0);
//     }

//     float kp=0.075;    
//     float kd=0, ki=0;
//     private:


//     float p(){
//         float sum=0;
//         for (int j=0;j<p_buffer.size(); j++){
//             sum+=p_buffer[j];
//         }
//         return sum;
//     }
//     int correction_rate=2; //Hz
//     int bufferSize= correction_rate*(FPS/MOTOR_CALLBACK);
//     std::vector <float>p_buffer=std::vector <float>(bufferSize,0);
//     float i=0, d=0;
//     float tolerance_upper=0.01, tolerance_lower=-0.01;

//     struct MedianFilter{
//         int kernelSize=3;
//         std::vector<float>buffer=std::vector<float>(kernelSize,0);

//         float get_median(){
//             std::vector <float> tmp=buffer;
//             std::sort(tmp.begin(), tmp.end());
//             return tmp[int(kernelSize/2)];
//         }
//     }mf;
    

// }correct;

// friend Task::Correct;    

// class ControlLearner{ //to learn wheel speed controls
//     private:
//     float weight=1.0;
// };


Task::Action getAction()const{
    return action;
}

AffordanceIndex getAffIndex(){
    return affordance;
}


Direction H(Disturbance, Direction, bool topDown=0); //topDown enables Configurator topdown control on reactive behaviour


void setEndCriteria(const Angle& angle=SAFE_ANGLE, const Distance& distance=BOX2DRANGE);

void setEndCriteria(const Distance& distance);

void setEndCriteria(const EndCriteria & ec){
    endCriteria=ec;
}


void setErrorWeights();

EndedResult checkEnded(b2Transform robotTransform= b2Transform_zero, Direction dir=UNDEFINED, bool relax=0, b2Body* robot=NULL, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform_zero));

EndedResult checkEnded(const State&, Direction dir=UNDEFINED, bool relax=false, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform_zero)); //usually used to check against control goal

/**
 * @brief Uses a virtual sensor (attention window) to determine whether the task has ended or not
 * 
 * @param box the sensor
 * @param robot_pose 
 * @param dist_obs pointer to the observed disturbance (the disturbance as it was at the beginning of the task, or as it was expected)
 * @return true 
 * @return false 
 */
bool checkEnded( const b2PolygonShape &box, const b2Transform& robot_pose=b2Transform_zero, Disturbance * dist_obs=NULL );

Task(){
    start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    direction = DEFAULT;
    action.init(direction);
}

Task(Direction d){
    direction=d;
    action.init(direction);
}

Task(Disturbance ob, Direction d, b2Transform _start=b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)), bool topDown=0){
    start = _start;
    disturbance = ob;
    affordance=disturbance.getAffIndex();
    direction = H(disturbance, d, topDown);  
    action.init(direction);
    setEndCriteria();
}


simResult bumping_that(b2World &, int, b2Body *, float remaining = SIM_DURATION);

EndCriteria getEndCriteria(const Disturbance&);

EndCriteria getEndCriteria(){
    return endCriteria;
}

bool endCriteria_met(Angle &, Distance &);

b2Transform from_Di( const b2Transform * custom_start=NULL, Disturbance * d_obs=NULL); //d_obs disturbance observed rather than D with which task was init

void set_change(bool b){
    change=b;
}

bool get_change(){
    return change;
}

/**
 * @brief Get a pointer to the disturbance
 * 
 * @return Disturbance* 
 */
Disturbance * get_disturbance_ptr(){
    return &disturbance;
}

const Disturbance & get_disturbance()const{
    return disturbance;
}

void setMotorStep(int i){
    motorStep=i;
}

int & getMotorStep(){return motorStep;}

b2Transform getStart(){
    return start;
}

b2Transform& getStartRef(){
    return start;
}

Direction get_direction(){
    return direction;
}

void set_direction(Direction d){
    direction=d;
}

protected:
    Action action;
    Disturbance disturbance;


};

#endif