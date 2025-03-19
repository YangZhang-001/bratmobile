#include "CloCK_math.h"

void math::applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose){
	pose =b2MulT(deltaPose, pose);
}

void math::applyAffineTrans(const b2Transform& deltaPose, State& state){
	applyAffineTrans(deltaPose, state.endPose);
	applyAffineTrans(deltaPose, state.start);
	if (state.Dn.getAffIndex()!=NONE){
		applyAffineTrans(deltaPose, state.Dn.bf.pose);
	}
	if (state.Di.getAffIndex()!=NONE){
		applyAffineTrans(deltaPose, state.Di.bf.pose);
	}

}

void math::applyAffineTrans(const b2Transform& deltaPose, Task* task){
	math::applyAffineTrans(-deltaPose, task->start);
	applyAffineTrans(-deltaPose, task->disturbance);
}



void math::applyAffineTrans(const b2Transform& deltaPose, TransitionSystem& g){
	auto vPair =boost::vertices(g);
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){ //each node is adjusted in explorer, so now we update
		if (*vIt!=0){
			math::applyAffineTrans(deltaPose, g[*vIt]);
		}
		else{
			math::applyAffineTrans(deltaPose, g[*vIt].Di);
			math::applyAffineTrans(deltaPose, g[*vIt].Dn);
		}
	}
}

void math::applyAffineTrans(const b2Transform& deltaPose, Disturbance& d){
	if (d.getAffIndex()!=NONE){
		math::applyAffineTrans(deltaPose, d.bf.pose);
	}
}

cv::Mat math::cv_affine_matrix33(const b2Transform & t){
	cv::Mat result=cv::getRotationMatrix2D(cv::Point2f(t.p.x, t.p.y), double(t.q.GetAngle()*(1/DEG_TO_RAD_K)), double(1));
	cv::Mat bottom_row=cv::Mat::zeros(1, 3, CV_32F);
	bottom_row.at<float>(1, 3)=1;
	result.push_back(bottom_row);
	return result;
	
}

b2Transform math::transform_2d(const cv::Mat & m){
	if (m.rows!=3 || m.cols!=3){
		throw std::invalid_argument("not 3x3 matrix");
	}                          //x                 //y                              //sin                //cos
	return b2Transform(b2Vec2(m.at<float>(1, 3), m.at<float>(2,3)), b2Rot(atan2(m.at<float>(2,1), m.at<float>(1,1))));
}

b2Transform math::solveAxB(const b2Transform& x, const b2Transform & B){ //
	cv::Mat x_matrix=math::cv_affine_matrix33(x); //x in matix form
    cv::Mat x_inv_matrix=x_matrix.inv(); //invert
    b2Transform x_inv= math::transform_2d(x_inv_matrix);
	return b2Mul(B, x_inv);
}
