#include "disturbance.h"

bool BodyFeatures::match(const BodyFeatures& bf, Bundle * bundle, b2Transform t)const{
    float hypothenuse_square= pow(bf.pose.p.Length(), 2); //assumes robot-centric perspective
    float adj_side_square=pow(bf.pose.p.Length()*t.q.c, 2);
    float distance_adjust= sqrt(hypothenuse_square-adj_side_square);
     float diff_x=pose.p.x -bf.pose.p.x;//-t.q.s*distance_adjust
    float diff_y=pose.p.y-bf.pose.p.y; //+t.q.c*distance_adjust
   //InvMul float diff_transform=bf.pose.p.Length()- pose.p.Length();
    float diff_w=halfWidth-bf.halfWidth;
    float diff_l=halfLength-bf.halfLength;
    bool match_x=fabs(diff_x)<D_POSE_MARGIN+ fabs(t.q.s*distance_adjust);
    bool match_y=fabs(diff_y)<D_POSE_MARGIN+fabs(t.q.c*distance_adjust);
    bool match_distance=pose.p.Length()-bf.pose.p.Length()<D_POSE_MARGIN;
    bool match_w=fabs(diff_w)<D_DIMENSIONS_MARGIN;
    bool match_h=fabs(diff_l)<D_DIMENSIONS_MARGIN;
    if (bundle!=NULL){
        *bundle=Bundle(diff_x, diff_y, 0, diff_w, diff_l);
    }
    //return match_x && match_y && match_w && match_h;
    return match_w && match_h && match_distance;
}

std::vector <b2Vec2> BodyFeatures::vertices()const{
    std::vector <b2Vec2> result;
    result.push_back(b2Vec2(halfWidth, halfLength));
    result.push_back(b2Vec2(halfWidth, -halfLength));
    result.push_back(b2Vec2(-halfWidth, halfLength));
    result.push_back(b2Vec2(-halfWidth, -halfLength)); //make upright box
    for (b2Vec2& v: result){
        v=b2Mul(pose, v);
    }
    return result;
}

std::vector <cv::Point2f> BodyFeatures::vertices_cv()const{
    std::vector <b2Vec2> vb2d=vertices();
    std::vector <cv::Point2f> result;
    for (const b2Vec2 & v: vb2d){
        result.push_back(cv::Point2f(v.x, v.y));
    }
    return result;
}



std::vector <b2Vec2> Disturbance::vertices()const{
    std::vector <b2Vec2> result;
    if (getAffIndex()==NONE){
        return result;
    }
    result= bf.vertices();
    return result;

}

float Disturbance::getAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = getPosition().x-t.p.x;
        thisToB.y = getPosition().y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angle = acos(cosA);
        return angle;
    }

// void Disturbance::setOrientation(float s, float c){
//     b2Rot og;
//     og.s=s;
//     og.c=c;
//     if (rotation_valid){
//     b2Rot sup, comp, ver; //find most likely angle
//         sup.s=-s;
//         sup.c=c;
//         comp.c=-c;
//         comp.s=s;
//         ver.s=-s;
//         ver.c=-c;
//         std::vector <b2Rot> rots={sup, comp, ver};
//         for (b2Rot r:rots){
//             if (fabs(r.GetAngle()-bf.pose.q.GetAngle())<fabs(og.GetAngle()-bf.pose.q.GetAngle())){
//                 og=r;
//             }
//         }
//     }
//     bf.pose.q.s=og.s;
//     bf.pose.q.c=og.c;
//     rotation_valid=1;
// }


// bool Disturbance::operator==(const Disturbance & d){
//     bool _pose=bf.pose.p==d.bf.pose.p && bf.pose.q.GetAngle()==d.bf.pose.q.GetAngle();
//     bool dim=halfLength()==d.bf.halfLength && halfWidth()==d.bf.halfWidth;
//     bool aff=affordanceIndex==d.affordanceIndex;
// }

bool Disturbance::operator==(const Disturbance & d)const{
    bool _position=std::round(bf.pose.p.x*100)/100==std::round(d.bf.pose.p.x*100)/100 && std::round(bf.pose.p.y*100)/100==std::round(d.bf.pose.p.y*100)/100;
    bool _pose=_position && (bf.pose.q.GetAngle()==d.bf.pose.q.GetAngle());
    bool dim=(bf.halfLength==d.bf.halfLength) && (bf.halfWidth==d.bf.halfWidth);
    bool aff=affordanceIndex==d.affordanceIndex;
    return _pose && dim && aff;
}

std::vector <b2Vec2> GetLocalPoints( std::vector <b2Vec2> pts, const b2Body * body){
	std::vector <b2Vec2> result;
	for (b2Vec2 p:pts){
		result.push_back(body->GetLocalPoint(p));
	}
	return result;
}