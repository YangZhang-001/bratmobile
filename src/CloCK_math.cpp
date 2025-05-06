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
	cv::Point2f p(t.p.x, t.p.y);
	double angle=double(t.q.GetAngle())*double(1/DEG_TO_RAD_K), scale=1.0; 
	cv::Mat result=cv::getRotationMatrix2D(p, angle, scale);
	// cv::Mat bottom_row=cv::Mat::zeros(1, 3, CV_32F);
	// bottom_row.at<float>(1, 3)=1;
	// result.push_back(bottom_row);
	return result;
	
}

b2Transform math::transform_2d(const cv::Mat & m){
	if (m.rows<2 || m.cols<3){
		throw std::invalid_argument("not 2x3 matrix");
	} 
	b2Transform result; 
	float x=m.at<double>(0, 2);
	float y=m.at<double>(1, 2);
	float s=m.at<double>(0,1);
	float c=m.at<double>(0,0);
	float t=atan2(-m.at<double>(0,1), m.at<double>(0,0));
	result.p.x=x;
	result.p.y=y;
	result.q.Set(t);
	return result;
}

b2Transform math::solveAxB(const b2Transform& x, const b2Transform & B){ //
	cv::Point2f p(x.p.x, x.p.y);
	double angle=double(x.q.GetAngle())*double(1/DEG_TO_RAD_K), scale=1.0; 
	cv::Mat x_matrix(3, 3, CV_32F);
	x_matrix.at<float>(0,0)=x.q.c;
	x_matrix.at<float>(1,1)=x.q.c;
	x_matrix.at<float>(0,1)=x.q.s;
	x_matrix.at<float>(1,0)=-x.q.s;
	x_matrix.at<float>(0,2)=x.p.x;
	x_matrix.at<float>(1,2)=x.p.y;
	x_matrix.at<float>(2,0)=0;
	x_matrix.at<float>(2,1)=0;
	x_matrix.at<float>(2,2)=1;
    cv::Mat x_inv_matrix=x_matrix.inv();
	//cv::invertAffineTransform(x_matrix, x_inv_matrix);
    b2Transform x_inv= math::transform_2d(x_inv_matrix);
	
	return b2Mul(B, x_inv);
}
