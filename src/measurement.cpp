#include "measurement.h"

bool Measurement::operator<(Measurement & m2){
    bool r = false;
    if (isValid() & m2.isValid()){
        r= get()<m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator<=(Measurement & m2){
    bool r = false;
    if (isValid() & m2.isValid()){
        r= get()<=m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator>=(Measurement &m2){
    bool r = false;
    if (isValid() & m2.isValid()){
        r= get()>=m2.get();
    }
    else{
        r=true;
    }
    return r;
}

bool Measurement::operator==(Measurement & m2){
    bool r=false;
    if (valid == m2.isValid() ){
        if (valid){
            r= get_signed()==m2.get_signed();
        }
        else {r=true;}
    }
    return r;
}



float Measurement::getStandardError(Measurement m2, float max){ 
    float result =0;
    if (m2.isValid()& this->isValid()){
        float num = get_signed()-m2.get_signed();
        if (num ==0){
            return result;
        }
        result = num/max;
    }
    return result;
}


float EndCriteria::getStandardError(Angle a, Distance d){ //standard error
    float result =0;
    float a_error=fabs(angle.getStandardError(a, MAX_ANGLE_ERROR));
    float d_error=fabs(distance.getStandardError(d, MAX_DISTANCE_ERROR));
    result = a_error +d_error; //max =2;
    return result/2; //return normalise
}


float EndCriteria::getStandardError(Angle a, Distance d, State n){
    float result =0;
    result = getStandardError(a, d); //+ weights[2]*outcomeError;
    if (n.filled & n.outcome== simResult::crashed){
        result+=2;
    }
    return result/3; //normalised to max value it can take
}



float SignedVectorLength(b2Vec2 v){
	float signedLength = v.Length();
	if (v.x <0){
		signedLength = -signedLength;
	}
	return signedLength;
}

bool EndCriteria::hasEnd(){
    return angle.isValid() || distance.isValid();
}

void EndCriteria::adjust(const b2Transform& delta){
    if (angle.isValid()){
        b2Rot newAngle=b2MulT(b2Rot(angle.get_signed()), delta.q);
        //angle.set(angle.get_signed()-delta.q.GetAngle());
       angle.set(newAngle.GetAngle());
    }
    if (distance.isValid()){
        distance.set(distance.get_signed()-delta.p.Length());
    }
}

