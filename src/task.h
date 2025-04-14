#ifndef TASK_H
#define TASK_H
#include "measurement.h"

b2Fixture * GetSensor(b2Body * body);

b2Body * GetDisturbance(b2World *);

//bool overlaps(b2Body *, b2Body *);

bool overlaps(b2Body *, Disturbance *);

bool overlaps(const b2PolygonShape&, Disturbance *, const b2Transform& robot_pose=b2Transform_zero);



class Task{


public:
    friend class Configurator;
    char planFile[250]; //for debug
    bool debug_k=false; //delete this it's for debugging on the bhenchod pi
    b2Transform start=b2Transform_zero;
    bool change =0;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction= DEFAULT;
    int motorStep=0;
   // int stepError=0;
    AffordanceIndex affordance=NONE;

struct Action{
private:
    float linearSpeed=WHEEL_SPEED_DEFAULT*2; //used to calculate instantaneous velocity using omega
  //  float recordedSpeed=linearSpeed;
    float omega=0; //initial angular velocity is 0
   // float recordedOmega = omega;
    bool valid=0;
public:
    float R=WHEEL_SPEED_DEFAULT;
    float L=WHEEL_SPEED_DEFAULT;

    Action()=default;

    void init(Direction& direction, float stim_intensity=0){
        stim_intensity=std::round(stim_intensity*100)/100;
        switch (direction){
        case Direction::DEFAULT:
        L=WHEEL_SPEED_DEFAULT;
        R=WHEEL_SPEED_DEFAULT;
        break;
        case Direction::LEFT:
        L=WHEEL_SPEED_DEFAULT-1*stim_intensity;
        R=WHEEL_SPEED_DEFAULT+1*stim_intensity;
        break;
        case Direction::RIGHT:
        L=WHEEL_SPEED_DEFAULT+1*stim_intensity;
        R=WHEEL_SPEED_DEFAULT-1*stim_intensity;
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

    float getRWheelSpeed(){
        return R;
    }

    float getLWheelSpeed(){
    return L;
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

    /*
    Expresses the relative position of disturbance as a float which is related to the error signal to it associated.Action
    This is used to tune wheel speed.
    */

};



class Listener : public b2ContactListener {
 // int iteration=1;
    Disturbance * d_ptr;
    public:
    Listener(){}
    Listener(Disturbance * _d): d_ptr(_d){}
    std::vector <b2Body*> collisions;
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

        
	};
	

public:
// friend Task::Correct;    

class ControlLearner{ //to learn wheel speed controls
    private:
    float weight=1.0;
};

Action action;

Disturbance disturbance;

Task::Action getAction(){
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


simResult bumping_that(b2World &, int, b2Body *, float remaining = SIM_DURATION);

EndCriteria getEndCriteria(const Disturbance&);

bool endCriteria_met(Angle &, Distance &);

b2Transform from_Di( const b2Transform * custom_start=NULL, Disturbance * d_obs=NULL); //d_obs disturbance observed rather than D with which task was init

std::pair <Angle, Distance> get_measurement(b2Transform t){
    std::pair<Angle, Distance> result(Angle(t.q.GetAngle()), Distance(t.p.Length()));
    return result;
}

};

#endif