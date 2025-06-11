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
    char planFile[250]; //for debug
    bool debug_k=false; //delete this it's for debugging on the bhenchod pi

public:
    friend class Configurator;
    b2Transform start=b2Transform_zero;
    bool change =0;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction= DEFAULT;
    AffordanceIndex affordance=NONE;

class Action{
    float linearSpeed=WHEEL_SPEED_DEFAULT*2; //used to calculate instantaneous velocity using omega
    float omega=0; //initial angular velocity is 0
    bool valid=0;
    float R=WHEEL_SPEED_DEFAULT;
    float L=WHEEL_SPEED_DEFAULT;
    int m_motorStep=0;

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
    linearSpeed = MAX_SPEED*(l+r)/2;
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

    void set_motorStep(int f){
        m_motorStep=f;
    }

    int motorStep()const{
        return m_motorStep;
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

public:



Disturbance disturbance;

Task::Action getAction()const{
    return action;
}

AffordanceIndex getAffIndex(){
    return affordance;
}


Direction H(Disturbance, Direction, bool topDown=0); //topDown enables Configurator topdown control on reactive behaviour


void setEndCriteria(const Angle& angle=SAFE_ANGLE, const Distance& distance=BOX2DRANGE);

void setEndCriteria(const Distance& distance);

void setErrorWeights();

EndedResult checkEnded(b2Transform robotTransform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)), Direction dir=UNDEFINED, bool relax=0, b2Body* robot=NULL, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0))));

EndedResult checkEnded(const State&, Direction dir=UNDEFINED, bool relax=false, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)))); //usually used to check against control goal

bool checkEnded( const b2PolygonShape &, const b2Transform& robot_pose=b2Transform_zero, Disturbance * dist_obs=NULL );

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

/**
 * @brief Simulates task and returns simulation results 
 * 
 * @param _world box2d world
 * @param iteration configurator iteration (for printing to file)
 * @param robot robot box2d body
 * @param remaining simulation time remaining in seconds
 * @return simResult 
 */
simResult bumping_that(b2World &_world, int iteration, b2Body * robot, float remaining = SIM_DURATION);

EndCriteria getEndCriteria(const Disturbance&);

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
Disturbance * get_disturbance(){
    return &disturbance;
}

private:

Action action;
};

#endif